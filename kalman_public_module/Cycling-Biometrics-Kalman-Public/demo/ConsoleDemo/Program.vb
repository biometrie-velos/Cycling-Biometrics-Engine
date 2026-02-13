Option Strict On
Option Explicit On
Option Infer On

Imports System
Imports System.Drawing
Imports CyclingBiometrics.Kalman

Module Program

    ' Synthetic demo:
    ' - build a noisy 2D trajectory for the 8 points
    ' - apply the Kalman smoother
    ' - print a few samples to show effect

    Sub Main()
        Dim settings As New KalmanSettings(imageWidth:=1280, imageHeight:=720)
        Using smoother As New KalmanSmoothers.KalmanSmoother8Bike(settings)

            Dim rnd As New Random(123)
            Dim pose As Pose8 = Pose8.EmptyPose()

            Dim x As Single = 200.0F
            Dim y As Single = 200.0F
            Dim vx As Single = 3.5F
            Dim vy As Single = 2.0F

            For i As Integer = 1 To 180
                ' ground truth motion
                x += vx
                y += vy
                If x < 50 OrElse x > 1230 Then vx = -vx
                If y < 50 OrElse y > 670 Then vy = -vy

                ' create noisy measurement (with occasional dropouts)
                Dim drop As Boolean = (i Mod 37 = 0) OrElse (i Mod 53 = 0)
                Dim meas As Point = If(drop, Point.Empty, New Point(
                    CInt(x + (rnd.NextDouble() * 14.0 - 7.0)),
                    CInt(y + (rnd.NextDouble() * 14.0 - 7.0))
                ))

                ' Use same meas for all points for demo simplicity
                pose.Metatarse = meas
                pose.Cheville = meas
                pose.Genou = meas
                pose.Hanche = meas
                pose.CreteIliaque = meas
                pose.Epaule = meas
                pose.Coude = meas
                pose.Poignee = meas

                smoother.Apply(pose, maxJumpPx:=60.0F, keepPredictedWhenMissing:=True)

                If i Mod 20 = 0 Then
                    Console.WriteLine($"Frame {i:000}  IN={(If(meas = Point.Empty, "EMPTY", meas.ToString()))}  OUT={pose.Genou}")
                End If
            Next
        End Using

        Console.WriteLine("Done. Press ENTER.")
        Console.ReadLine()
    End Sub

End Module
