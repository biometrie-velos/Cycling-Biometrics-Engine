Option Strict On
Option Explicit On
Option Infer On

Imports System.Drawing

Namespace CyclingBiometrics.Kalman

    ''' <summary>
    ''' Smoothers operating on the public DTO (Pose8).
    ''' Point.Empty means "missing measurement".
    ''' Output is "measurement if good, Kalman only if gate triggers".
    ''' </summary>
    Public Module KalmanSmoothers

#Region "──────── Smoother 4 points JAMBE (GATED) ────────"

        Public NotInheritable Class KalmanSmoother4Leg
            Implements IDisposable

            Private ReadOnly _settings As KalmanSettings
            Private ReadOnly _f(3) As KalmanCore.Kalman2DBike
            Private ReadOnly _clk As New KalmanCore.DtClock()

            Private _gm As KalmanCore.GateState
            Private _gc As KalmanCore.GateState
            Private _gg As KalmanCore.GateState
            Private _gh As KalmanCore.GateState

            Private Const BAD_TO_ENABLE As Integer = 2
            Private Const GOOD_TO_DISABLE As Integer = 5

            Public Sub New(settings As KalmanSettings)
                If settings Is Nothing Then Throw New ArgumentNullException(NameOf(settings))
                _settings = settings

                For i As Integer = 0 To 3
                    _f(i) = New KalmanCore.Kalman2DBike(_settings) With {.AllowEmptyBeforeFirstFix = False}
                Next
                ResetAll()
            End Sub

            Public Sub ResetAll()
                For i As Integer = 0 To 3
                    _f(i).ResetAll()
                Next
                _clk.Reset()

                _gm = New KalmanCore.GateState()
                _gc = New KalmanCore.GateState()
                _gg = New KalmanCore.GateState()
                _gh = New KalmanCore.GateState()
            End Sub

            Public Sub ApplyLegOnly(ByRef pose As Pose8,
                                    Optional maxJumpPx As Single = 60.0F,
                                    Optional keepPredictedWhenMissing As Boolean = True)

                Dim dt As Single = _clk.GetDtSeconds()

                Dim mIn As Point = KalmanCore.ClampPoint(pose.Metatarse, _settings)
                Dim cIn As Point = KalmanCore.ClampPoint(pose.Cheville, _settings)
                Dim gIn As Point = KalmanCore.ClampPoint(pose.Genou, _settings)
                Dim hIn As Point = KalmanCore.ClampPoint(pose.Hanche, _settings)

                KalmanCore.UpdateGate(_gm, mIn, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)
                KalmanCore.UpdateGate(_gc, cIn, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)
                KalmanCore.UpdateGate(_gg, gIn, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)
                KalmanCore.UpdateGate(_gh, hIn, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)

                Dim mF As PointF = _f(0).StepKalman(mIn, mIn <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)
                Dim cF As PointF = _f(1).StepKalman(cIn, cIn <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)
                Dim gF As PointF = _f(2).StepKalman(gIn, gIn <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)
                Dim hF As PointF = _f(3).StepKalman(hIn, hIn <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)

                Dim mK As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(mF, _settings), If(mIn = Point.Empty, _gm.LastGoodMeas, mIn), _settings)
                Dim cK As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(cF, _settings), If(cIn = Point.Empty, _gc.LastGoodMeas, cIn), _settings)
                Dim gK As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(gF, _settings), If(gIn = Point.Empty, _gg.LastGoodMeas, gIn), _settings)
                Dim hK As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(hF, _settings), If(hIn = Point.Empty, _gh.LastGoodMeas, hIn), _settings)

                pose.Metatarse = KalmanCore.GatePick(mIn, mK, _gm.UseKalman)
                pose.Cheville = KalmanCore.GatePick(cIn, cK, _gc.UseKalman)
                pose.Genou = KalmanCore.GatePick(gIn, gK, _gg.UseKalman)
                pose.Hanche = KalmanCore.GatePick(hIn, hK, _gh.UseKalman)
            End Sub

            Public Sub Dispose() Implements IDisposable.Dispose
                For i As Integer = 0 To 3
                    _f(i).Dispose()
                Next
            End Sub

        End Class

#End Region

#Region "──────── Smoother 8 points (GATED) ────────"

        Public NotInheritable Class KalmanSmoother8Bike
            Implements IDisposable

            Private ReadOnly _settings As KalmanSettings
            Private ReadOnly _f(7) As KalmanCore.Kalman2DBike
            Private ReadOnly _clk As New KalmanCore.DtClock()

            Private _g0, _g1, _g2, _g3, _g4, _g5, _g6, _g7 As KalmanCore.GateState

            Private Const BAD_TO_ENABLE As Integer = 2
            Private Const GOOD_TO_DISABLE As Integer = 5

            Public Sub New(settings As KalmanSettings)
                If settings Is Nothing Then Throw New ArgumentNullException(NameOf(settings))
                _settings = settings

                For i As Integer = 0 To 7
                    _f(i) = New KalmanCore.Kalman2DBike(_settings) With {.AllowEmptyBeforeFirstFix = False}
                Next
                ResetAll()
            End Sub

            Public Sub ResetAll()
                For i As Integer = 0 To 7
                    _f(i).ResetAll()
                Next
                _clk.Reset()

                _g0 = New KalmanCore.GateState()
                _g1 = New KalmanCore.GateState()
                _g2 = New KalmanCore.GateState()
                _g3 = New KalmanCore.GateState()
                _g4 = New KalmanCore.GateState()
                _g5 = New KalmanCore.GateState()
                _g6 = New KalmanCore.GateState()
                _g7 = New KalmanCore.GateState()
            End Sub

            Public Sub Apply(ByRef pose As Pose8,
                             Optional maxJumpPx As Single = 60.0F,
                             Optional keepPredictedWhenMissing As Boolean = True)

                Dim dt As Single = _clk.GetDtSeconds()

                Dim p0 As Point = KalmanCore.ClampPoint(pose.Metatarse, _settings)
                Dim p1 As Point = KalmanCore.ClampPoint(pose.Cheville, _settings)
                Dim p2 As Point = KalmanCore.ClampPoint(pose.Genou, _settings)
                Dim p3 As Point = KalmanCore.ClampPoint(pose.Hanche, _settings)
                Dim p4 As Point = KalmanCore.ClampPoint(pose.CreteIliaque, _settings)
                Dim p5 As Point = KalmanCore.ClampPoint(pose.Epaule, _settings)
                Dim p6 As Point = KalmanCore.ClampPoint(pose.Coude, _settings)
                Dim p7 As Point = KalmanCore.ClampPoint(pose.Poignee, _settings)

                KalmanCore.UpdateGate(_g0, p0, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)
                KalmanCore.UpdateGate(_g1, p1, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)
                KalmanCore.UpdateGate(_g2, p2, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)
                KalmanCore.UpdateGate(_g3, p3, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)
                KalmanCore.UpdateGate(_g4, p4, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)
                KalmanCore.UpdateGate(_g5, p5, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)
                KalmanCore.UpdateGate(_g6, p6, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)
                KalmanCore.UpdateGate(_g7, p7, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)

                Dim f0 As PointF = _f(0).StepKalman(p0, p0 <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)
                Dim f1 As PointF = _f(1).StepKalman(p1, p1 <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)
                Dim f2 As PointF = _f(2).StepKalman(p2, p2 <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)
                Dim f3 As PointF = _f(3).StepKalman(p3, p3 <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)
                Dim f4 As PointF = _f(4).StepKalman(p4, p4 <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)
                Dim f5 As PointF = _f(5).StepKalman(p5, p5 <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)
                Dim f6 As PointF = _f(6).StepKalman(p6, p6 <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)
                Dim f7 As PointF = _f(7).StepKalman(p7, p7 <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)

                Dim k0 As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(f0, _settings), If(p0 = Point.Empty, _g0.LastGoodMeas, p0), _settings)
                Dim k1 As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(f1, _settings), If(p1 = Point.Empty, _g1.LastGoodMeas, p1), _settings)
                Dim k2 As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(f2, _settings), If(p2 = Point.Empty, _g2.LastGoodMeas, p2), _settings)
                Dim k3 As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(f3, _settings), If(p3 = Point.Empty, _g3.LastGoodMeas, p3), _settings)
                Dim k4 As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(f4, _settings), If(p4 = Point.Empty, _g4.LastGoodMeas, p4), _settings)
                Dim k5 As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(f5, _settings), If(p5 = Point.Empty, _g5.LastGoodMeas, p5), _settings)
                Dim k6 As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(f6, _settings), If(p6 = Point.Empty, _g6.LastGoodMeas, p6), _settings)
                Dim k7 As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(f7, _settings), If(p7 = Point.Empty, _g7.LastGoodMeas, p7), _settings)

                pose.Metatarse = KalmanCore.GatePick(p0, k0, _g0.UseKalman)
                pose.Cheville = KalmanCore.GatePick(p1, k1, _g1.UseKalman)
                pose.Genou = KalmanCore.GatePick(p2, k2, _g2.UseKalman)
                pose.Hanche = KalmanCore.GatePick(p3, k3, _g3.UseKalman)
                pose.CreteIliaque = KalmanCore.GatePick(p4, k4, _g4.UseKalman)
                pose.Epaule = KalmanCore.GatePick(p5, k5, _g5.UseKalman)
                pose.Coude = KalmanCore.GatePick(p6, k6, _g6.UseKalman)
                pose.Poignee = KalmanCore.GatePick(p7, k7, _g7.UseKalman)
            End Sub

            Public Sub Dispose() Implements IDisposable.Dispose
                For i As Integer = 0 To 7
                    _f(i).Dispose()
                Next
            End Sub

        End Class

#End Region

#Region "──────── Smoother 4 points MEMBRES SUP (GATED) ────────"

        Public NotInheritable Class KalmanSmootherUpper4
            Implements IDisposable

            Private ReadOnly _settings As KalmanSettings
            Private ReadOnly _f(3) As KalmanCore.Kalman2DBike
            Private ReadOnly _clk As New KalmanCore.DtClock()

            Private _gP5 As KalmanCore.GateState
            Private _gP6 As KalmanCore.GateState
            Private _gP7 As KalmanCore.GateState
            Private _gP8 As KalmanCore.GateState

            Private Const BAD_TO_ENABLE As Integer = 2
            Private Const GOOD_TO_DISABLE As Integer = 5

            Public Sub New(settings As KalmanSettings)
                If settings Is Nothing Then Throw New ArgumentNullException(NameOf(settings))
                _settings = settings

                For i As Integer = 0 To 3
                    _f(i) = New KalmanCore.Kalman2DBike(_settings) With {.AllowEmptyBeforeFirstFix = False}
                Next
                ResetAll()
            End Sub

            Public Sub ResetAll()
                For i As Integer = 0 To 3
                    _f(i).ResetAll()
                Next
                _clk.Reset()

                _gP5 = New KalmanCore.GateState()
                _gP6 = New KalmanCore.GateState()
                _gP7 = New KalmanCore.GateState()
                _gP8 = New KalmanCore.GateState()
            End Sub

            ''' <summary>
            ''' Apply Kalman only on upper-body points (CreteIliaque, Epaule, Coude, Poignee).
            ''' Lower-body points are not modified.
            ''' </summary>
            Public Sub ApplyUpperOnly(ByRef pose As Pose8,
                                     Optional maxJumpPx As Single = 60.0F,
                                     Optional keepPredictedWhenMissing As Boolean = True)

                Dim dt As Single = _clk.GetDtSeconds()

                Dim p5 As Point = KalmanCore.ClampPoint(pose.CreteIliaque, _settings)
                Dim p6 As Point = KalmanCore.ClampPoint(pose.Epaule, _settings)
                Dim p7 As Point = KalmanCore.ClampPoint(pose.Coude, _settings)
                Dim p8 As Point = KalmanCore.ClampPoint(pose.Poignee, _settings)

                KalmanCore.UpdateGate(_gP5, p5, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)
                KalmanCore.UpdateGate(_gP6, p6, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)
                KalmanCore.UpdateGate(_gP7, p7, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)
                KalmanCore.UpdateGate(_gP8, p8, maxJumpPx, BAD_TO_ENABLE, GOOD_TO_DISABLE)

                Dim f5 As PointF = _f(0).StepKalman(p5, p5 <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)
                Dim f6 As PointF = _f(1).StepKalman(p6, p6 <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)
                Dim f7 As PointF = _f(2).StepKalman(p7, p7 <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)
                Dim f8 As PointF = _f(3).StepKalman(p8, p8 <> Point.Empty, dt, maxJumpPx, keepPredictedWhenMissing)

                Dim k5 As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(f5, _settings), If(p5 = Point.Empty, _gP5.LastGoodMeas, p5), _settings)
                Dim k6 As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(f6, _settings), If(p6 = Point.Empty, _gP6.LastGoodMeas, p6), _settings)
                Dim k7 As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(f7, _settings), If(p7 = Point.Empty, _gP7.LastGoodMeas, p7), _settings)
                Dim k8 As Point = KalmanCore.ToPointNoHole(KalmanCore.ClampPointF(f8, _settings), If(p8 = Point.Empty, _gP8.LastGoodMeas, p8), _settings)

                pose.CreteIliaque = KalmanCore.GatePick(p5, k5, _gP5.UseKalman)
                pose.Epaule = KalmanCore.GatePick(p6, k6, _gP6.UseKalman)
                pose.Coude = KalmanCore.GatePick(p7, k7, _gP7.UseKalman)
                pose.Poignee = KalmanCore.GatePick(p8, k8, _gP8.UseKalman)
            End Sub

            Public Sub Dispose() Implements IDisposable.Dispose
                For i As Integer = 0 To 3
                    _f(i).Dispose()
                Next
            End Sub

        End Class

#End Region

    End Module

End Namespace
