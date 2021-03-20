close all;
clear;

% ROI extraction parameters
ROI_EXTRACTION_METHOD = 'adaptth-moore'; % adaptth-moore, canny-dfs, canny-dfs-c
ADAPTTH_SENSITIVITY = 0.7;        % [0,1]
ADAPTTH_STATISTIC = 'gaussian';   % mean, gaussian, median
ADAPTTH_NEIGHBORHOOD = [135 241]; % default for 1080x1920 -> [135 241]
CANNY_TH_LOW  = 0.01;
CANNY_TH_HIGH = 0.10;

% ROI refinement parameters
ROI_REFINEMENT_METHOD = 'rdp'; % rdp, geometric
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
ROI_EXTRACTION_VERBOSE = 0; % 1: show roi_extracted  2: + adaptth/canny 
ROI_REFINEMENT_VERBOSE = 0; % 1: show roi_refined    2: + roi_discarded
ROI_MATCHING_VERBOSE   = 1; % 1: show roi_matched    2: + log roi_matched  3: + log roi_refined
ARUCO_DETECTION_VERBOSE = 0; % TODO

% Open image
% img1 = imread('../assets/img_tests/web.jpg');
img1 = imread('../assets/img_tests/test_iPhone/File_002.jpeg');
img2 = imread('../assets/img_tests/test_iPhone/File_006.jpeg');
% img1 = imread('../assets/img_tests/test1/images1_01.png');
% img2 = imread('../assets/img_tests/test1/images2_01.png');
% img1 = imread('../assets/img_tests/test6/images1_01.png');
% img2 = imread('../assets/img_tests/test6/images2_01.png');
% img1 = imread('../assets/img_tests/test7/images1_01.png');
% img2 = imread('../assets/img_tests/test7/images2_01.png');
% img1 = imread('../assets/img_tests/test8/images1_01.png');
% img2 = imread('../assets/img_tests/test8/images2_01.png');
% img1 = imread('../assets/img_tests/test9/img1.png');
% img1 = imread('../assets/img_tests/test9/img2.png');
% img2 = imread('../assets/img_tests/test9/img3.png');

% Load examples of markers
% load('../assets/aruco_markers/aruco_markers_8x8_web.mat'); % test web
% load('../assets/aruco_markers/aruco_markers_8x8_iPhone.mat'); % test_iPhone
% load('../assets/aruco_markers/aruco_markers_7x7.mat'); % test1
load('../assets/aruco_markers/aruco_markers_8x8_camera.mat'); % test6-7-8-9

% Wrap parameters
parameters.roi_extraction_method = ROI_EXTRACTION_METHOD;
parameters.adaptth_sensitivity = ADAPTTH_SENSITIVITY;
parameters.adaptth_statistic = ADAPTTH_STATISTIC;
parameters.adaptth_neighborhood = ADAPTTH_NEIGHBORHOOD;
parameters.canny_th_low = CANNY_TH_LOW;
parameters.canny_th_high = CANNY_TH_HIGH;
parameters.roi_refinement_method = ROI_REFINEMENT_METHOD;
parameters.roi_size_th = ROI_SIZE_TH;
parameters.rdp_th = RDP_TH;
parameters.roi_sum_angles_tol = ROI_SUM_ANGLES_TOL;
parameters.roi_parallelism_tol = ROI_PARALLELISM_TOL;
parameters.roi_side_th_low = ROI_SIDE_TH_LOW;
parameters.roi_side_th_high = ROI_SIDE_TH_HIGH;
parameters.roi_bb_padding = ROI_BB_PADDING;
parameters.roi_h_side = ROI_H_SIDE;
parameters.roi_hamming_th = ROI_HAMMING_TH;
parameters.roi_extraction_verbose = ROI_EXTRACTION_VERBOSE;
parameters.roi_refinement_verbose = ROI_REFINEMENT_VERBOSE;
parameters.roi_matching_verbose = ROI_MATCHING_VERBOSE;
parameters.verbose = ARUCO_DETECTION_VERBOSE;

fprintf('------- Processing 1st image -------\n');
[rois_matched1, i_arucos1] = aruco_detection(img1, aruco_markers, parameters);

fprintf('------- Processing 2nd image -------\n');
[rois_matched2, i_arucos2] = aruco_detection(img2, aruco_markers, parameters);
