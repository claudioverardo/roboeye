%% Acquire calibration images (from 2 cameras)
% cam1 = webcam(1);
% cam2 = webcam(2);
% preview(cam1);
% preview(cam2);
n_images    = 20;
cameras     = {cam1, cam2};
dirs_images = {'../assets/calibration/intrinsics_cam1', '../assets/calibration/intrinsics_cam2'};

% images = acquire_calibration_images(n_images, cameras, dirs_images);

%% Calculate the intrinsics of the two cameras with SMZ
clear; close all;

dir_images1 = '../assets/calibration/intrinsics_cam1';
dir_images2 = '../assets/calibration/intrinsics_cam2';
n_intrinsics  = 5;  % num internal parameters (4 or 5)
n_radial_dist = 1;  % num radial distortion coefficients (1 or 2)
cm2px_scale = 0.05; % dimension in cm of 1 pixel of the rectified image

% [P1, K1, intrinsics1] = calibration_intrinsics_camera(dir_images1, n_intrinsics, n_radial_dist, cm2px_scale);
% [P2, K2, intrinsics2] = calibration_intrinsics_camera(dir_images2, n_intrinsics, n_radial_dist, cm2px_scale);

%% Calculate the extrinsics of the two cameras wrt a checkerboard
close all; clear;

cam1 = webcam(1);
load('../assets/calibration/intrinsics_cam1/K.mat'); K1 = K; clear('K');
dir1 = '../assets/calibration/extrinsics_cam1';
cm2px_scale = 0.05;
[R1, t1] = calibration_extrinsics_camera(cam1, K1, dir1, cm2px_scale);

%% Calculate the relative pose of the two cameras from SMZ data
clear; close all;

load('../assets/calibration/intrinsics_cam1/P.mat'); P1 = P; clear('P');
load('../assets/calibration/intrinsics_cam1/K.mat'); K1 = K; clear('K');
load('../assets/calibration/intrinsics_cam2/P.mat'); P2 = P; clear('P');
load('../assets/calibration/intrinsics_cam2/K.mat'); K2 = K; clear('K');
dir = '../assets/calibration/extrinsics_stereo_cam12';

[delta_R, delta_t, E, F] = calibration_extrinsics_stereo(P1, K1, P2, K2, dir);
