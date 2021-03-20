close all
clear
 
% ROI extraction parameters
ROI_EXTRACTION_METHOD = 'adaptth-moore'; % adaptth-moore, canny-dfs, canny-dfs-c
ADAPTTH_SENSITIVITY = 0.7;        % [0,1]
ADAPTTH_STATISTIC = 'gaussian';   % mean, gaussian, median
ADAPTTH_NEIGHBORHOOD = [135 241]; % default for 1080x1920 -> [135 241]
CANNY_TH_LOW  = 0.01;
CANNY_TH_HIGH = 0.10;

% ROI refinement parameters
ROI_REFINEMENT_METHOD = 'geometric'; % rdp, geometric
ROI_SIZE_TH = 50;
RDP_TH = 0.1;  % Ramer–Douglas–Peucker threshold
ROI_SUM_ANGLES_TOL  = 10; % [degrees]
ROI_PARALLELISM_TOL = 20; % [degrees]
ROI_SIDE_TH_LOW  = 1/100; % [% diag(img)]
ROI_SIDE_TH_HIGH = 1/5;   % [% diag(img)]

% ROI matching parameters
ROI_BB_PADDING  = 2;
ROI_H_SIDE = 80;
ROI_HAMMING_TH  = 3;

% Debug/Analysis
ROI_EXTRACTION_VERBOSE        = 0; % 1: show roi_extracted  2: + adaptth/canny 
ROI_REFINEMENT_VERBOSE        = 0; % 1: show roi_refined    2: + roi_discarded
ROI_MATCHING_VERBOSE          = 0; % 1: show roi_matched    2: + log roi_matched  3: + log roi_refined
ROI_POSE_ESTIMATION_VERBOSE   = 2; % 1: show roi_pnp        2: + aruco id/error
ARUCO_DETECTION_VERBOSE       = 0; % TODO
ARUCO_POSE_ESTIMATION_VERBOSE = 0; % TODO

%--------- TEST img_tests9 (from camera) ----------

% cam = webcam(1);
% img = snapshot(cam);
% img = imread('../assets/img_tests/test9/img1.png'); % 2 aruco
% img = imread('../assets/img_tests/test9/img2.png'); % 3 aruco
img = imread('../assets/img_tests/test9/img3.png'); % 4 aruco

aruco_real_sides = [3 3 6 6]; % [cm]
load('../assets/aruco_markers/aruco_markers_8x8_camera.mat');
load('../assets/calibration/intrinsics_cam1/K.mat', 'K');
load('../assets/calibration/intrinsics_cam1/intrinsics.mat', 'intrinsics');
load('../assets/calibration/extrinsics_cam1/R.mat', 'R');
load('../assets/calibration/extrinsics_cam1/t.mat', 't');

%---------- TEST img_tests6-8 (old setup) -----------

% img = imread('../assets/img_tests/test6/images1_01.png');
% img = imread('../assets/img_tests/test7/images1_01.png');
% img = imread('../assets/img_tests/test8/images1_01.png');
% img = imread('../assets/img_tests/test8/images2_01.png');

% aruco_real_sides = [3 3]; % [cm]
% load('../assets/aruco_markers/aruco_markers_8x8_camera.mat');
% load('../assets/calibration/K_old.mat', 'K');

%---------- TEST img_tests iPhone ----------

% Load image
% img = imread('../assets/img_tests/test_iPhone/File_001.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_002.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_003.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_004.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_005.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_006.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_007.jpeg');
% img = imresize(img, [900 1200]);

% aruco_real_sides = 6; % [cm]
% load('../assets/aruco_markers/aruco_markers_8x8_iPhone.mat');
% load('../assets/calibration/K_iPhone.mat');

%---------------------------------

K = K'; % intrinsics from Fusiello toolkit
R = R';
t = t';

% Camera intrinsics object
% K(1,1) = -K(1,1);
% K(2,2) = -K(2,2);
% K(1,2) = -K(1,2);
% focal_length    = [K(1,1) K(2,2)]; 
% principal_point = [K(3,1) K(3,2)];
% image_size      = [size(img,2), size(img,1)];
% skew            = K(2,1);
% K_obj = cameraIntrinsics(focal_length, principal_point, image_size, 'Skew', skew);

% Launch Pose Estimation w/ PnP
[rois, i_arucos, rois_R, rois_t] = aruco_pose_estimation( ...
    img, aruco_markers, aruco_real_sides, K, R, t, ...
    'roi_extraction_method', ROI_EXTRACTION_METHOD, ...
    'adaptth_sensitivity', ADAPTTH_SENSITIVITY, ...
    'adaptth_statistic', ADAPTTH_STATISTIC, ...
    'adaptth_neighborhood', ADAPTTH_NEIGHBORHOOD, ...
    'canny_th_low', CANNY_TH_LOW, ...
    'canny_th_high', CANNY_TH_HIGH, ...
    'roi_refinement_method', ROI_REFINEMENT_METHOD, ...
    'roi_size_th', ROI_SIZE_TH, ...
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
    'roi_pose_estimation_verbose', ROI_POSE_ESTIMATION_VERBOSE, ...
    'aruco_detection_verbose', ARUCO_DETECTION_VERBOSE, ...
    'verbose', ARUCO_POSE_ESTIMATION_VERBOSE ...
);

