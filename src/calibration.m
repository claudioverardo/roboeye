%% Acquire calibration images
% cam1 = webcam(1);
% cam2 = webcam(2);
cameras     = {cam1, cam2};
dirs_images = {'../assets/calibration/01', '../assets/calibration/02'};
images = acquire_calibration_images(20, cameras, dirs_images);

%% Calculate P and K matrices of the two cameras
[P1, K1, intrinsics1] = calibration_camera(20, '../assets/calibration/01');
[P2, K2, intrinsics2] = calibration_camera(20, '../assets/calibration/02');

%% Calculate stereo calibration paramenters
load('../assets/calibration/01/P.mat'); P1 = P; clear('P');
load('../assets/calibration/02/P.mat'); P2 = P; clear('P');
load('../assets/calibration/01/K.mat'); K1 = K; clear('K');
load('../assets/calibration/02/K.mat'); K2 = K; clear('K');
[deltaT, E, F] = calibration_stereo(P1, K1, P2, K2);
