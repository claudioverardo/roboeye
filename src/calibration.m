%% Acquire calibration images
% cam1 = webcam(1);
% cam2 = webcam(2);
num_images  = 20;
cameras     = {cam1, cam2};
dirs_images = {'../assets/calibration/01', '../assets/calibration/02'};
images = acquire_calibration_images(num_images, cameras, dirs_images);

%% Calculate P and K matrices of the two cameras
close all;
[P1, K1, intrinsics1] = calibration_camera(20, '../assets/calibration/01');
[P2, K2, intrinsics2] = calibration_camera(20, '../assets/calibration/02');

%% Calculate stereo calibration paramenters
% [F, E, deltaT] = calibration_stereo(P1, P2);