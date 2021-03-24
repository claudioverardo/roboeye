%% Create camera objects
cam1 = webcam(1);
cam2 = webcam(2);
% preview(cam1);
% preview(cam2);

%% Acquire images for SMZ calibration (from 2 cameras)
close all;

n_images    = 20;
cameras     = {cam1, cam2};
dirs_images = {'../assets/calibration/intrinsics_cam1', '../assets/calibration/intrinsics_cam2'};

% images = acquire_calibration_images(n_images, cameras, dirs_images);

%% Calculate the intrinsics/distortion parameters of the 2 cameras with SMZ
close all;

dir_images1 = '../assets/calibration/intrinsics_cam1';
dir_images2 = '../assets/calibration/intrinsics_cam2';
n_intrinsics  = 5;
n_radial_dist = 1;
step_size = 3;
grid_arrangement = [8, 6];
cm2px_scale = 0.05;

% [P1, K1, intrinsics1] = calibration_intrinsics_camera(n_intrinsics, n_radial_dist, step_size, grid_arrangement, cm2px_scale, dir_images1);
% [P2, K2, intrinsics2] = calibration_intrinsics_camera(n_intrinsics, n_radial_dist, step_size, grid_arrangement, cm2px_scale, dir_images2);

%% Calculate the extrinsics of the 2 cameras wrt a checkerboard
close all;

step_size = 3;
grid_arrangement = [16, 10];
cm2px_scale = 0.05;

load('../assets/calibration/intrinsics_cam1/K.mat'); K1 = K; clear('K');
dir1 = '../assets/calibration/extrinsics_cam1';
% [R_cam1, t_cam1] = calibration_extrinsics_camera(cam1, K1, step_size, grid_arrangement, cm2px_scale, dir1);

load('../assets/calibration/intrinsics_cam2/K.mat'); K2 = K; clear('K');
dir2 = '../assets/calibration/extrinsics_cam2';
% [R_cam2, t_cam2] = calibration_extrinsics_camera(cam2, K2, step_size, grid_arrangement, cm2px_scale, dir2);

%% Calculate the relative pose of the 2 cameras from their intrisics and extrinsics
close all;

load('../assets/calibration/intrinsics_cam1/K.mat'); K1 = K; clear('K');
load('../assets/calibration/extrinsics_cam1/R_cam.mat'); R1 = R_cam; clear('R_cam');
load('../assets/calibration/extrinsics_cam1/t_cam.mat'); t1 = t_cam; clear('t_cam');
load('../assets/calibration/intrinsics_cam2/K.mat'); K2 = K; clear('K');
load('../assets/calibration/extrinsics_cam2/R_cam.mat'); R2 = R_cam; clear('R_cam');
load('../assets/calibration/extrinsics_cam2/t_cam.mat'); t2 = t_cam; clear('t_cam');
dir = '../assets/calibration/extrinsics_stereo_cam12';

% [delta_R, delta_t, E, F] = calibration_extrinsics_stereo(K1, R1, t1, K2, R2, t2, dir);

% Test stereo calibration
% check_epipolar_geometry(cam1, cam2, F);

%% Calculate the relative pose of the 2 cameras from SMZ output
close all;

load('../assets/calibration/intrinsics_cam1/P.mat'); P1 = P; clear('P');
load('../assets/calibration/intrinsics_cam1/K.mat'); K1 = K; clear('K');
load('../assets/calibration/intrinsics_cam2/P.mat'); P2 = P; clear('P');
load('../assets/calibration/intrinsics_cam2/K.mat'); K2 = K; clear('K');
dir = '../assets/calibration/extrinsics_stereo_cam12_smz';

% [delta_R_smz, delta_t_smz, E_smz, F_smz] = calibration_extrinsics_stereo_smz(P1, K1, P2, K2, dir);

% Test stereo calibration
% check_epipolar_geometry(cam1, cam2, F_smz);
