%% Acquire images for SMZ calibration (from 2 cameras)
clear; close all;

cam1 = webcam(1);
cam2 = webcam(2);
% preview(cam1);
% preview(cam2);
n_images    = 20;
cameras     = {cam1, cam2};
dirs_images = {'../assets/calibration/intrinsics_cam1', '../assets/calibration/intrinsics_cam2'};

% images = acquire_calibration_images(n_images, cameras, dirs_images);


%% Calculate the intrinsics/distortion parameters of the 2 cameras with SMZ
clear; close all;

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
clear; close all;

cam1 = webcam(1);
load('../assets/calibration/intrinsics_cam1/K.mat'); K1 = K; clear('K');
dir1 = '../assets/calibration/extrinsics_cam1';
step_size = 3;
grid_arrangement = [8, 10];
cm2px_scale = 0.05;

[R_cam1, t_cam1] = calibration_extrinsics_camera(cam1, K1, step_size, grid_arrangement, cm2px_scale, dir1);


%% Calculate the relative pose of the 2 cameras from SMZ output
clear; close all;

load('../assets/calibration/intrinsics_cam1/P.mat'); P1 = P; clear('P');
load('../assets/calibration/intrinsics_cam1/K.mat'); K1 = K; clear('K');
load('../assets/calibration/intrinsics_cam2/P.mat'); P2 = P; clear('P');
load('../assets/calibration/intrinsics_cam2/K.mat'); K2 = K; clear('K');
dir = '../assets/calibration/extrinsics_stereo_cam12';

% [delta_R, delta_t, E, F] = calibration_extrinsics_stereo(P1, K1, P2, K2, dir);
