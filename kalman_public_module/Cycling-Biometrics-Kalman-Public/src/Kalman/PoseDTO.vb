Option Strict On
Option Explicit On
Option Infer On

Imports System.Drawing

Namespace CyclingBiometrics.Kalman

    ''' <summary>
    ''' Public DTO for an 8-point cycling pose (2D).
    ''' Use Point.Empty to represent missing measurements.
    ''' </summary>
    Public Structure Pose8
        Public Metatarse As Point
        Public Cheville As Point
        Public Genou As Point
        Public Hanche As Point
        Public CreteIliaque As Point
        Public Epaule As Point
        Public Coude As Point
        Public Poignee As Point

        Public Shared Function EmptyPose() As Pose8
            Return New Pose8() With {
                .Metatarse = Point.Empty,
                .Cheville = Point.Empty,
                .Genou = Point.Empty,
                .Hanche = Point.Empty,
                .CreteIliaque = Point.Empty,
                .Epaule = Point.Empty,
                .Coude = Point.Empty,
                .Poignee = Point.Empty
            }
        End Function
    End Structure

    ''' <summary>
    ''' Shared settings for clamping / validity constraints.
    ''' </summary>
    Public NotInheritable Class KalmanSettings
        Public ReadOnly Property ImageWidth As Integer
        Public ReadOnly Property ImageHeight As Integer

        Public Sub New(imageWidth As Integer, imageHeight As Integer)
            If imageWidth <= 0 Then Throw New ArgumentOutOfRangeException(NameOf(imageWidth))
            If imageHeight <= 0 Then Throw New ArgumentOutOfRangeException(NameOf(imageHeight))
            Me.ImageWidth = imageWidth
            Me.ImageHeight = imageHeight
        End Sub

        Public ReadOnly Property XMin As Single
            Get
                Return 0.0F
            End Get
        End Property

        Public ReadOnly Property YMin As Single
            Get
                Return 0.0F
            End Get
        End Property

        Public ReadOnly Property XMax As Single
            Get
                Return CSng(ImageWidth - 1)
            End Get
        End Property

        Public ReadOnly Property YMax As Single
            Get
                Return CSng(ImageHeight - 1)
            End Get
        End Property
    End Class

End Namespace
