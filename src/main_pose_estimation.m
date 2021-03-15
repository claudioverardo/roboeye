close all
clear

% Image binarization
ADAPTTH_SENSITIVITY = 1; % [0,1]
ADAPTTH_STATISTIC = 'mean'; % mean, gaussian, median

% ROI extraction parameters
ROI_EXTRACTION_METHOD = 'adaptth-moore'; % adaptth-moore, canny-dfs, canny-dfs-c
CANNY_TH_LOW  = 0.01;
CANNY_TH_HIGH = 0.10;

% ROI refinement parameters
ROI_REFINEMENT_METHOD = 'rdp'; % rdp, geometric
RDP_TH = 0.1;  % Ramer–Douglas–Peucker threshold
ROI_SUM_ANGLES_TOL  = 10; % [degrees]
ROI_PARALLELISM_TOL = 15; % [degrees]
ROI_SIDE_TH_LOW  = 20;    % [pixels]
ROI_SIDE_TH_HIGH = 700;   % [pixels]

% ROI matching parameters
ROI_BB_PADDING  = 2;
ROI_H_SIDE = 80;
ROI_HAMMING_TH  = 1;

% Debug/Analysis
ROI_EXTRACTION_VERBOSE = 1;
ROI_REFINEMENT_VERBOSE = 1;
ROI_MATCHING_VERBOSE   = 1;
ROI_PNP_VERBOSE        = 1;
ARUCO_DETECTION_VERBOSE       = 0;
ARUCO_POSE_ESTIMATION_VERBOSE = 0;

% img = imread('../assets/img_tests/test6/images1_01.png');
% img = imread('../assets/img_tests/test7/images1_01.png');
img = imread('../assets/img_tests/test8/images1_01.png');
% img = imread('../assets/img_tests/test8/images2_01.png');

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

% Camera intrinsics object
K = K'; % intrinsics from Fusiello toolkit
K(1,1) = -K(1,1);
K(2,2) = -K(2,2);
focal_length    = [K(1,1) K(2,2)]; 
principal_point = [K(3,1) K(3,2)];
image_size      = [size(img,2), size(img,1)];
skew            = K(2,1);
K_obj = cameraIntrinsics(focal_length, principal_point, image_size, 'Skew', skew);

% Launch Pose Estimation w/ PnP
[rois, i_arucos, R, t] = aruco_pose_estimation( ...
    img, aruco_markers, aruco_real_side, K_obj, ...
    'adaptth_sensitivity', ADAPTTH_SENSITIVITY, ...
    'adaptth_statistic', ADAPTTH_STATISTIC, ...
    'roi_extraction_method', ROI_EXTRACTION_METHOD, ...
    'canny_th_low', CANNY_TH_LOW, ...
    'canny_th_high', CANNY_TH_HIGH, ...
    'roi_refinement_method', ROI_REFINEMENT_METHOD, ...
    'rdp_th', RDP_TH, ...
    'roi_sum_angles_tol', ROI_SUM_ANGLES_TOL, ...
    'roi_parallelism_tol', ROI_PARALLELISM_TOL, ...
    'roi_side_th_low', ROI_SIDE_TH_LOW, ...
    'roi_side_th_high', ROI_SIDE_TH_HIGH, ...
    'roi_bb_padding', ROI_BB_PADDING, ...
    'roi_h_side', ROI_H_SIDE, ...
    'roi_hamming_th', ROI_HAMMING_TH, ...
    'roi_extraction_verbose', ROI_EXTRACTION_VERBOSE, ...
    'roi_refinement_verbose', ROI_REFINEMENT_VERBOSE, ...
    'roi_matching_verbose', ROI_MATCHING_VERBOSE, ...
    'roi_pnp_verbose', ROI_PNP_VERBOSE, ...
    'aruco_detection_verbose', ARUCO_DETECTION_VERBOSE, ...
    'verbose', ARUCO_POSE_ESTIMATION_VERBOSE ...
);

