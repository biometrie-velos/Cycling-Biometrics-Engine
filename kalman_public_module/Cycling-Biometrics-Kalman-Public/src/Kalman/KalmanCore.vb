Option Strict On
Option Explicit On
Option Infer On

Imports System.Diagnostics
Imports System.Drawing
Imports System.Runtime.InteropServices
Imports Emgu.CV
Imports Emgu.CV.CvEnum
Imports Emgu.CV.Structure

Namespace CyclingBiometrics.Kalman

    Public Module KalmanCore

#Region "──────── Helpers mémoire (Mat <-> Single[]) ────────"

        Friend NotInheritable Class MatF32
            Private Sub New()
            End Sub

            Public Shared Sub Write(m As Mat, buf() As Single, count As Integer)
                If m Is Nothing OrElse buf Is Nothing Then Exit Sub
                If m.IsEmpty Then Exit Sub
                Marshal.Copy(buf, 0, m.DataPointer, count)
            End Sub

            Public Shared Sub Read(m As Mat, buf() As Single, count As Integer)
                If m Is Nothing OrElse buf Is Nothing Then Exit Sub
                If m.IsEmpty Then Exit Sub
                Marshal.Copy(m.DataPointer, buf, 0, count)
            End Sub
        End Class

        Friend Function ClampF(v As Single, minV As Single, maxV As Single) As Single
            If v < minV Then Return minV
            If v > maxV Then Return maxV
            Return v
        End Function

        Friend Function ClampPointF(p As PointF, s As KalmanSettings) As PointF
            Return New PointF(
                ClampF(p.X, s.XMin, s.XMax),
                ClampF(p.Y, s.YMin, s.YMax)
            )
        End Function

        Friend Function ClampPoint(p As Point, s As KalmanSettings) As Point
            If p = Point.Empty Then Return p
            Dim x As Integer = CInt(ClampF(p.X, s.XMin, s.XMax))
            Dim y As Integer = CInt(ClampF(p.Y, s.YMin, s.YMax))
            Return New Point(x, y)
        End Function

        Friend Function ToPointNoHole(v As PointF, fallback As Point, s As KalmanSettings) As Point
            If v = PointF.Empty Then Return ClampPoint(fallback, s)
            Return ClampPoint(Point.Round(v), s)
        End Function

#End Region

#Region "──────── Gate (tracking quality + hysteresis) ────────"

        ''' <summary>
        ''' Petit état persistant par point (pour le gating)
        ''' </summary>
        Friend Structure GateState
            Public LastGoodMeas As Point
            Public GoodStreak As Integer
            Public BadStreak As Integer
            Public UseKalman As Boolean
        End Structure

        Friend Function IsBadMeas(meas As Point, lastGood As Point, maxJumpPx As Single) As Boolean
            If meas = Point.Empty Then Return True
            If lastGood = Point.Empty Then Return False ' pas de référence -> on accepte
            If maxJumpPx <= 0.0F OrElse Single.IsNaN(maxJumpPx) OrElse Single.IsInfinity(maxJumpPx) Then Return False

            Dim dx As Integer = meas.X - lastGood.X
            Dim dy As Integer = meas.Y - lastGood.Y
            Dim d2 As Integer = dx * dx + dy * dy
            Dim thr As Integer = CInt(maxJumpPx * maxJumpPx)
            Return d2 > thr
        End Function

        ''' <summary>
        ''' Hystérésis simple : bascule vers Kalman après N bad, retour vers brut après M good
        ''' </summary>
        Friend Sub UpdateGate(ByRef st As GateState,
                              meas As Point,
                              maxJumpPx As Single,
                              badToEnable As Integer,
                              goodToDisable As Integer)

            Dim bad As Boolean = IsBadMeas(meas, st.LastGoodMeas, maxJumpPx)

            If bad Then
                st.BadStreak += 1
                st.GoodStreak = 0
            Else
                st.GoodStreak += 1
                st.BadStreak = 0
                st.LastGoodMeas = meas
            End If

            If Not st.UseKalman Then
                If st.BadStreak >= badToEnable Then st.UseKalman = True
            Else
                If st.GoodStreak >= goodToDisable Then st.UseKalman = False
            End If
        End Sub

        Friend Function GatePick(meas As Point, kal As Point, useKalman As Boolean) As Point
            Return If(useKalman, kal, meas)
        End Function

#End Region

#Region "──────── Horloge dt unique ────────"

        Friend NotInheritable Class DtClock
            Private ReadOnly _sw As Stopwatch = Stopwatch.StartNew()
            Private _lastTick As Long

            Public Sub Reset()
                _lastTick = 0
            End Sub

            Public Function GetDtSeconds() As Single
                Dim t As Long = _sw.ElapsedTicks
                If _lastTick = 0 Then
                    _lastTick = t
                    Return 1.0F / 30.0F
                End If

                Dim dt As Single = CSng((t - _lastTick) / CDbl(Stopwatch.Frequency))
                _lastTick = t

                If dt <= 0.0F OrElse Single.IsNaN(dt) OrElse Single.IsInfinity(dt) Then Return 1.0F / 30.0F
                If dt < 0.001F Then Return 0.001F
                If dt > 0.1F Then Return 0.1F
                Return dt
            End Function
        End Class

#End Region

#Region "──────── Kalman 2D (6D) : x,y,vx,vy,ax,ay ────────"

        ''' <summary>
        ''' 2D Kalman filter with a 6D state:
        ''' x, y, vx, vy, ax, ay and a 2D measurement (x, y).
        ''' </summary>
        Public NotInheritable Class Kalman2DBike
            Implements IDisposable

            Private ReadOnly _settings As KalmanSettings

            Private ReadOnly _kf As KalmanFilter
            Private ReadOnly _meas As Mat

            Private ReadOnly _x(5) As Single
            Private ReadOnly _z(1) As Single

            Private ReadOnly _A(35) As Single
            Private ReadOnly _Q(35) As Single
            Private ReadOnly _P(35) As Single
            Private ReadOnly _H(11) As Single
            Private ReadOnly _R(3) As Single

            Private _init As Boolean
            Private _lastX As Single
            Private _lastY As Single

            Private _hasOut As Boolean
            Private _outX As Single
            Private _outY As Single

            Private _lostTime As Single

            ' Defaults (same tuning as your original module)
            Private Const Q_POS As Single = 0.12F
            Private Const Q_VEL As Single = 0.35F
            Private Const Q_ACC As Single = 0.9F
            Private Const R_BASE As Single = 6.0F
            Private Const P0_POS As Single = 220.0F
            Private Const P0_VEL As Single = 120.0F
            Private Const P0_ACC As Single = 80.0F
            Private Const DAMP_V_MISS As Single = 0.9992F
            Private Const DAMP_A_MISS As Single = 0.9985F
            Private Const VMAX As Single = 4000.0F
            Private Const LOST_TIMEOUT_S As Single = 10.0F

            Public Property AllowEmptyBeforeFirstFix As Boolean = False

            Public Sub New(settings As KalmanSettings)
                If settings Is Nothing Then Throw New ArgumentNullException(NameOf(settings))
                _settings = settings

                _kf = New KalmanFilter(6, 2, 0, DepthType.Cv32F)
                _meas = New Mat(2, 1, DepthType.Cv32F, 1)

                BuildConstantMatrices()
                ResetAll()
            End Sub

            Private Sub BuildConstantMatrices()
                _H(0) = 1 : _H(1) = 0 : _H(2) = 0 : _H(3) = 0 : _H(4) = 0 : _H(5) = 0
                _H(6) = 0 : _H(7) = 1 : _H(8) = 0 : _H(9) = 0 : _H(10) = 0 : _H(11) = 0

                Array.Clear(_Q, 0, _Q.Length)
                _Q(0) = Q_POS
                _Q(7) = Q_POS
                _Q(14) = Q_VEL
                _Q(21) = Q_VEL
                _Q(28) = Q_ACC
                _Q(35) = Q_ACC

                _R(0) = R_BASE : _R(1) = 0
                _R(2) = 0 : _R(3) = R_BASE

                Array.Clear(_P, 0, _P.Length)
                _P(0) = P0_POS
                _P(7) = P0_POS
                _P(14) = P0_VEL
                _P(21) = P0_VEL
                _P(28) = P0_ACC
                _P(35) = P0_ACC
            End Sub

            Public Sub ResetAll()
                ResetInternal(preserveLastOut:=False)
                _hasOut = False
                _outX = 0.0F
                _outY = 0.0F
            End Sub

            Private Sub ResetInternal(preserveLastOut As Boolean)
                _kf.StatePost.SetTo(New MCvScalar(0))
                _kf.MeasurementMatrix.SetTo(New MCvScalar(0))
                _kf.ProcessNoiseCov.SetTo(New MCvScalar(0))
                _kf.MeasurementNoiseCov.SetTo(New MCvScalar(0))
                _kf.ErrorCovPost.SetTo(New MCvScalar(0))
                _kf.TransitionMatrix.SetTo(New MCvScalar(0))

                MatF32.Write(_kf.MeasurementMatrix, _H, 12)
                MatF32.Write(_kf.ProcessNoiseCov, _Q, 36)
                MatF32.Write(_kf.MeasurementNoiseCov, _R, 4)
                MatF32.Write(_kf.ErrorCovPost, _P, 36)

                _init = False
                _lostTime = 0.0F
                _lastX = 0.0F
                _lastY = 0.0F

                If Not preserveLastOut Then
                    _hasOut = False
                    _outX = 0.0F
                    _outY = 0.0F
                End If
            End Sub

            Private Sub UpdateA(dt As Single)
                If Single.IsNaN(dt) OrElse Single.IsInfinity(dt) OrElse dt <= 0.0F Then dt = 1.0F / 30.0F
                If dt < 0.001F Then dt = 0.001F
                If dt > 0.1F Then dt = 0.1F

                Dim dt2 As Single = 0.5F * dt * dt
                Array.Clear(_A, 0, _A.Length)

                _A(0) = 1.0F : _A(2) = dt : _A(4) = dt2
                _A(7) = 1.0F : _A(9) = dt : _A(11) = dt2
                _A(14) = 1.0F : _A(16) = dt
                _A(21) = 1.0F : _A(23) = dt
                _A(28) = 1.0F
                _A(35) = 1.0F

                MatF32.Write(_kf.TransitionMatrix, _A, 36)
            End Sub

            Private Sub StoreOut(x As Single, y As Single)
                _outX = ClampF(x, _settings.XMin, _settings.XMax)
                _outY = ClampF(y, _settings.YMin, _settings.YMax)
                _hasOut = True
            End Sub

            Private Function GetLastOutOrZero() As PointF
                If _hasOut Then Return New PointF(_outX, _outY)
                Return New PointF(0.0F, 0.0F)
            End Function

            Public Function StepKalman(pt As Point,
                                       valid As Boolean,
                                       dt As Single,
                                       maxJumpPx As Single,
                                       keepPredictedWhenMissing As Boolean) As PointF

                If Single.IsNaN(dt) OrElse Single.IsInfinity(dt) OrElse dt <= 0.0F Then dt = 1.0F / 30.0F
                If dt < 0.001F Then dt = 0.001F
                If dt > 0.1F Then dt = 0.1F

                UpdateA(dt)

                If Not _init Then
                    If Not valid Then
                        If AllowEmptyBeforeFirstFix Then Return PointF.Empty
                        Return GetLastOutOrZero()
                    End If

                    Dim p0 As Point = ClampPoint(pt, _settings)

                    Array.Clear(_x, 0, _x.Length)
                    _x(0) = p0.X
                    _x(1) = p0.Y

                    MatF32.Write(_kf.StatePost, _x, 6)

                    _init = True
                    _lostTime = 0.0F
                    _lastX = _x(0)
                    _lastY = _x(1)
                    StoreOut(_x(0), _x(1))
                    Return New PointF(_x(0), _x(1))
                End If

                If valid AndAlso maxJumpPx > 0.0F AndAlso
                   Not Single.IsNaN(maxJumpPx) AndAlso Not Single.IsInfinity(maxJumpPx) Then

                    Dim pm As Point = ClampPoint(pt, _settings)
                    Dim dxm As Single = CSng(pm.X) - _lastX
                    Dim dym As Single = CSng(pm.Y) - _lastY
                    Dim d2m As Single = (dxm * dxm) + (dym * dym)
                    Dim thr2 As Single = maxJumpPx * maxJumpPx

                    If d2m > thr2 Then
                        _x(0) = pm.X
                        _x(1) = pm.Y
                        _x(2) = 0.0F
                        _x(3) = 0.0F
                        _x(4) = 0.0F
                        _x(5) = 0.0F

                        MatF32.Write(_kf.StatePost, _x, 6)

                        _lostTime = 0.0F
                        _lastX = _x(0)
                        _lastY = _x(1)
                        StoreOut(_x(0), _x(1))
                        Return New PointF(_x(0), _x(1))
                    End If
                End If

                Dim pred As Mat = _kf.Predict()
                MatF32.Read(pred, _x, 6)

                _x(0) = ClampF(_x(0), _settings.XMin, _settings.XMax)
                _x(1) = ClampF(_x(1), _settings.YMin, _settings.YMax)

                If Not valid Then
                    _lostTime += dt

                    _x(2) *= DAMP_V_MISS
                    _x(3) *= DAMP_V_MISS
                    _x(4) *= DAMP_A_MISS
                    _x(5) *= DAMP_A_MISS

                    _x(2) = ClampF(_x(2), -VMAX, VMAX)
                    _x(3) = ClampF(_x(3), -VMAX, VMAX)

                    MatF32.Write(_kf.StatePost, _x, 6)

                    _lastX = _x(0)
                    _lastY = _x(1)

                    Dim outP As PointF = If(keepPredictedWhenMissing, New PointF(_x(0), _x(1)), GetLastOutOrZero())
                    outP = ClampPointF(outP, _settings)
                    StoreOut(outP.X, outP.Y)

                    If _lostTime >= LOST_TIMEOUT_S Then
                        ResetInternal(preserveLastOut:=True)
                    End If

                    Return outP
                End If

                Dim pMeas As Point = ClampPoint(pt, _settings)

                Dim vxEst As Single = (CSng(pMeas.X) - _lastX) / dt
                Dim vyEst As Single = (CSng(pMeas.Y) - _lastY) / dt
                If Single.IsNaN(vxEst) OrElse Single.IsInfinity(vxEst) Then vxEst = 0.0F
                If Single.IsNaN(vyEst) OrElse Single.IsInfinity(vyEst) Then vyEst = 0.0F

                vxEst = ClampF(vxEst, -VMAX, VMAX)
                vyEst = ClampF(vyEst, -VMAX, VMAX)

                _x(2) = vxEst
                _x(3) = vyEst
                MatF32.Write(_kf.StatePost, _x, 6)

                _z(0) = pMeas.X
                _z(1) = pMeas.Y
                MatF32.Write(_meas, _z, 2)

                Dim corr As Mat = _kf.Correct(_meas)
                MatF32.Read(corr, _x, 6)

                _x(0) = ClampF(_x(0), _settings.XMin, _settings.XMax)
                _x(1) = ClampF(_x(1), _settings.YMin, _settings.YMax)

                _lostTime = 0.0F
                _lastX = _x(0)
                _lastY = _x(1)

                StoreOut(_x(0), _x(1))
                Return New PointF(_x(0), _x(1))
            End Function

            Public Sub Dispose() Implements IDisposable.Dispose
                _meas.Dispose()
                _kf.Dispose()
            End Sub

        End Class

#End Region

    End Module

End Namespace
