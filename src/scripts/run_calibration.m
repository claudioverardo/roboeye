%% Acquire calibration images (from 2 cameras)
% cam1 = webcam(1);
% cam2 = webcam(2);
n_images    = 3;
cameras     = {cam1, cam2};
dirs_images = {'../assets/calibration/intrinsics_cam1', '../assets/calibration/intrinsics_cam2'};
images = acquire_calibration_images(n_images, cameras, dirs_images);

%% Calculate the intrinsics of the two cameras with SMZ
n_intrinsics  = 5; % number of internal parameters (typ. 4 or 5)
n_radial_dist = 1; % number of radial distortion coefficients (typ. 1 or 2).
[P1, K1, intrinsics1] = calibration_intrinsics_camera('../assets/calibration/intrinsics_cam1', n_intrinsics, n_radial_dist);
[P2, K2, intrinsics2] = calibration_intrinsics_camera('../assets/calibration/intrinsics_cam2', n_intrinsics, n_radial_dist);

%% Calculate the relative pose of the two cameras from SMZ data
load('../assets/calibration/intrinsics_cam1/P.mat'); P1 = P; clear('P');
load('../assets/calibration/intrinsics_cam1/K.mat'); K1 = K; clear('K');
load('../assets/calibration/intrinsics_cam2/P.mat'); P2 = P; clear('P');
load('../assets/calibration/intrinsics_cam2/K.mat'); K2 = K; clear('K');
[delta_R, delta_t, E, F] = calibration_extrinsics_stereo(P1, K1, P2, K2, '../assets/calibration/extrinsics_stereo_cam12');

%% Calculate the extrinsics of the two cameras wrt a checkerboard

