# Cycling Biometrics Engine — Kalman Module (Public)

This package is a **publishable** / **reusable** version of the Kalman smoothing module used in Cycling Biometrics pipelines.

## What's included
- `src/Kalman/PoseDTO.vb` — public DTO (`Pose8`) used by smoothers (no dependency on internal pipeline types).
- `src/Kalman/KalmanCore.vb` — core Kalman 2D (6D state) + gating + dt clock.
- `src/Kalman/KalmanSmoothers.vb` — smoothers (Leg4 / Upper4 / Bike8) operating on `Pose8`.
- `demo/ConsoleDemo` — a minimal .NET 8 VB.NET console demo (synthetic noisy trajectory).

## Dependencies
This code uses **EmguCV** for `KalmanFilter` + `Mat`.
- You need EmguCV runtime compatible with your platform.
- The demo project includes `PackageReference` entries you can adapt to your EmguCV version.

## Build (demo)
```bash
cd demo/ConsoleDemo
dotnet restore
dotnet run
```

## Notes
- Image size is configurable via `KalmanSettings(width, height)`.
- The API is designed so you can plug this module into any tracker that outputs 2D points.
