%% Acquire calibration images
num_images  = 3;
cameras     = {webcam(1), webcam(2)};
dirs_images = {'../assets/calibration/01', '../assets/calibration/02'};
images = acquire_calibration_images(num_images, cameras, dirs_images);

%% Calculate P and K matrices of the two cameras
[P1, K1] = calibration_camera('../assets/calibration/01', 5);
% [P2, K2] = calibration_camera('../assets/calibration/02', 15);

%% Calculate stereo calibration paramenters
% [F, E, deltaT] = calibration_stereo(P1, P2);