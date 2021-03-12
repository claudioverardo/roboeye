close all
clear

% ROIs extraction parameters
CANNY_TH_LOW  = 0.01;
CANNY_TH_HIGH = 0.10;
RDP_TH        = 0.1; % Ramer–Douglas–Peucker
ROI_SIZE_TH_LOW  = 20*20;   % [pixels^2]
ROI_SIZE_TH_HIGH = 700*700; % [pixels^2]
SUM_ANGLES_TOLERANCE  = 10; % [degrees]
PARALLELISM_TOLERANCE = 15; % [degrees]

% Aruco matching parameters
BB_PADDING  = 2;
ROI_H_SIDE  = 80;
HAMMING_TH  = 1;

% Debug
EXTRACT_ROI_VERBOSE = 0;
MATCH_ROI_VERBOSE   = 1;
ARUCO_DETECTION_VERBOSE = 2;
ARUCO_POSE_ESTIMATION_PNP_VERBOSE = 1;

% img = imread('../assets/img_tests/test6/images1_01.png');
% img = imread('../assets/img_tests/test7/images1_01.png');
img = imread('../assets/img_tests/test8/images1_01.png');

aruco_real_side = 0.03; % [m]
load('aruco_markers_8x8.mat');
load('data/K_P1.mat', 'K');


%---------- TEST iPhone ----------

% Load image
% img = imread('../assets/img_tests/test_iPhone/File_001.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_002.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_003.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_004.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_005.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_006.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_007.jpeg');
% img = imresize(img, [900 1200]);

% aruco_real_side = 0.06; % [m]
% load('aruco_markers_8x8_iPhone.mat');
% load('data/K_iPhone.mat');

%---------------------------------

% Wrap parameters
aruco_detection_parameters = {
    'canny_th_low' CANNY_TH_LOW
    'canny_th_high' CANNY_TH_HIGH
    'rdp_th' RDP_TH
    'roi_size_th_low' ROI_SIZE_TH_LOW
    'roi_size_th_high' ROI_SIZE_TH_HIGH
    'sum_angles_tolerance' SUM_ANGLES_TOLERANCE
    'parallelism_tolerance' PARALLELISM_TOLERANCE
    'bb_padding' BB_PADDING
    'roi_h_side' ROI_H_SIDE
    'hamming_th' HAMMING_TH
    'extract_roi_verbose' EXTRACT_ROI_VERBOSE
    'match_roi_verbose' MATCH_ROI_VERBOSE
    'verbose', ARUCO_DETECTION_VERBOSE
}';

% Launch Pose Estimation w/ PnP
[rois, i_arucos, R, t] = aruco_pose_estimation_pnp( ...
    img, K, aruco_markers, aruco_real_side, ...
    aruco_detection_parameters, ...
    'verbose', ARUCO_POSE_ESTIMATION_PNP_VERBOSE ...
);

