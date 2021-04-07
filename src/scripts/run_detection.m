close all; % clear

%% TESTS - DETECTION - ARUCO MARKERS 7x7
%--------------------------------------------------------------------------

config_file = '../assets/config_files/config_detection';
aruco_markers_file = '../assets/aruco_markers/aruco_markers_7x7.mat';

% img_source = '../assets/img_tests/7x7/img_7x7_04_01.png';
% img_source = '../assets/img_tests/7x7/img_7x7_04_02.png';
% img_source = '../assets/img_tests/7x7/img_7x7_04_03.png';
% img_source = '../assets/img_tests/7x7/img_7x7_06_01.png';
% img_source = '../assets/img_tests/7x7/img_7x7_06_02.png';
% img_source = '../assets/img_tests/7x7/img_7x7_06_03.png';
% img_source = '../assets/img_tests/7x7/img_7x7_08_01.png';
% img_source = '../assets/img_tests/7x7/img_7x7_08_02.png';
% img_source = '../assets/img_tests/7x7/img_7x7_12_01.png';
img_source = '../assets/img_tests/7x7/img_7x7_12_02.png';


%% TESTS - DETECTION - ARUCO MARKERS 8x8
%--------------------------------------------------------------------------

% config_file = '../assets/config_files/config_detection';
% aruco_markers_file = '../assets/aruco_markers/aruco_markers_8x8.mat';

% img_source = '../assets/img_tests/8x8/img_8x8_04_01.png';
% img_source = '../assets/img_tests/8x8/img_8x8_04_02.png';
% img_source = '../assets/img_tests/8x8/img_8x8_04_03.png';
% img_source = '../assets/img_tests/8x8/img_8x8_04_04.png';
% img_source = '../assets/img_tests/8x8/img_8x8_06_01.png';
% img_source = '../assets/img_tests/8x8/img_8x8_06_02.png';
% img_source = '../assets/img_tests/8x8/img_8x8_06_03.png';
% img_source = '../assets/img_tests/8x8/img_8x8_06_04.png';
% img_source = '../assets/img_tests/8x8/img_8x8_08_01.png';
% img_source = '../assets/img_tests/8x8/img_8x8_08_02.png';
% img_source = '../assets/img_tests/8x8/img_8x8_08_03.png';


%% TESTS - DETECTION (TEMPLATE)
%--------------------------------------------------------------------------

% config_file = 'path/to/config/file.m';
% aruco_markers_file = 'path/to/aruco/markers.mat';

% img_source = ...; % webcam(1) or 'path/to/image'


%% DO NOT CHANGE BELOW
% -------------------------------------------------------------------------

run(config_file);
load(aruco_markers_file);
img = get_image(img_source);

% Launch Aruco Detection
[rois_matched1, i_arucos1] = aruco_detection( ...
    img, aruco_markers, ...
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
    'roi_angle_th_low', ROI_ANGLE_TH_LOW, ...
    'roi_angle_th_high', ROI_ANGLE_TH_HIGH, ...
    'roi_bb_padding', ROI_BB_PADDING, ...
    'roi_h_side', ROI_H_SIDE, ...
    'roi_hamming_th', ROI_HAMMING_TH, ...
    'roi_extraction_verbose', ROI_EXTRACTION_VERBOSE, ...
    'roi_refinement_verbose', ROI_REFINEMENT_VERBOSE, ...
    'roi_matching_verbose', ROI_MATCHING_VERBOSE, ...
    'verbose', ARUCO_DETECTION_VERBOSE ...
);
