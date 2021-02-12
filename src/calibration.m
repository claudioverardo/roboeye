% Calculate P and K matrices of the two cameras
[P1, K1] = calibration_camera('../assets/calibration/01', 19);
[P2, K2] = calibration_camera('../assets/calibration/02', 19);

% Calculate stereo calibration paramenters
[F, E, deltaT] = calibration_stereo(P1, P2);