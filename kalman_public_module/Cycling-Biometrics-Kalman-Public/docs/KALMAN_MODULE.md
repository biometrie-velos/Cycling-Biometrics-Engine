# Kalman Module (Public)

## API
- `Pose8` DTO: 8 points (Point.Empty = missing)
- `KalmanSettings(width, height)`: image bounds for clamping
- Smoothers:
  - `KalmanSmoother8Bike.Apply(ByRef pose As Pose8, ...)`
  - `KalmanSmoother4Leg.ApplyLegOnly(ByRef pose As Pose8, ...)`
  - `KalmanSmootherUpper4.ApplyUpperOnly(ByRef pose As Pose8, ...)`

## Notes
The output policy is:
- Use raw measurement when it looks stable
- Switch to Kalman only when the gate detects repeated bad measurements (hysteresis)
