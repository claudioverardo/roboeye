%% Create camera object
cam = webcam(1);
% preview(cam);

%% Acquire images for SMZ calibration of the camera
close all;

n_images    = 20;
cameras     = {cam};
dirs_images = {'../assets/calibration/intrinsics_cam1'};

% images = acquire_calibration_images(n_images, cameras, dirs_images);

%% Calculate the intrinsics/distortion parameters of the camera with SMZ
close all;

dir_images = '../assets/calibration/intrinsics_cam1';
n_intrinsics  = 5;
n_radial_dist = 1;
step_size = 3;
grid_arrangement = [8, 6];
cm2px_scale = 0.05;

% [P, K, intrinsics] = calibration_intrinsics_camera(n_intrinsics, n_radial_dist, step_size, grid_arrangement, cm2px_scale, dir_images);

%% Calculate the extrinsics of the camera wrt a checkerboard
close all;

step_size = 3;
grid_arrangement = [16, 10];
cm2px_scale = 0.05;
load('../assets/calibration/intrinsics_cam1/K.mat');
load('../assets/calibration/intrinsics_cam1/intrinsics.mat');
k = intrinsics.radial;
% dir = '../assets/calibration/extrinsics_cam1_tests_7x7';
% dir = '../assets/calibration/extrinsics_cam1_tests_8x8';
% dir = '../assets/calibration/extrinsics_cam1_tests_robot';

% [R_cam, t_cam] = calibration_extrinsics_camera(cam, K, k, step_size, grid_arrangement, cm2px_scale, dir);
