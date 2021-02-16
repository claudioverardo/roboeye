clear;
close all;

% ROIs extraction parameters
CANNY_TH_LOW  = 0.01;
CANNY_TH_HIGH = 0.10;
RDP_TH        = 0.1; % Ramer�Douglas�Peucker

% Aruco matching parameters
BB_PADDING  = 2;
ROI_TH_SIZE = 0.5;
ROI_H_SIDE  = 80;
HAMMING_TH  = 1;

% Debug
EXTRACT_ROI_VERBOSE = 0;
MATCH_ROI_VERBOSE   = 1;

% Open image
% img1 = imread('../assets/img_tests/test1/images1_01.png');
% img2 = imread('../assets/img_tests/test1/images2_01.png');
% img1 = imread('../assets/img_tests/test2/images1_01.png');
% img2 = imread('../assets/img_tests/test2/images2_01.png');
% img1 = imread('../assets/img_tests/test3/images1_01.png');
% img2 = imread('../assets/img_tests/test3/images2_01.png');
img1 = imread('../assets/img_tests/test8/images1_01.png');
img2 = imread('../assets/img_tests/test8/images2_01.png');
% img1 = imread('../assets/img_tests/aruco.jpg');

% Load examples of markers
% load('aruco_markers_7x7.mat');
load('aruco_markers_8x8.mat');
% load('aruco_markers_8x8_web.mat');

%-----------------------------------------------------------------

% aruco_markers = logical(aruco_markers);
n_aruco_markers = size(aruco_markers,1);

figure;
for i=1:n_aruco_markers
    subplot(1,n_aruco_markers,i)
    imshow(aruco_markers{i});
    title(sprintf('query %d', i));
end

fprintf('------- Processing 1st image -------\n');
[rois_found1, i_rois1, i_arucos1, k_rots1] = aruco_detection(...
    img1, aruco_markers, ...
    'canny_th_low', CANNY_TH_LOW, ...
    'canny_th_high', CANNY_TH_HIGH, ...
    'rdp_th', RDP_TH, ...
    'bb_padding', BB_PADDING, ...
    'roi_th_size', ROI_TH_SIZE, ...
    'roi_h_side', ROI_H_SIDE, ...
    'hamming_th', HAMMING_TH, ...
    'extract_roi_verbose', EXTRACT_ROI_VERBOSE, ...
    'match_roi_verbose', MATCH_ROI_VERBOSE ...
);

fprintf('------- Processing 2nd image -------\n');
[rois_found2, i_rois2, i_arucos2, k_rots2] = aruco_detection(...
    img2, aruco_markers, ...
    'canny_th_low', CANNY_TH_LOW, ...
    'canny_th_high', CANNY_TH_HIGH, ...
    'rdp_th', RDP_TH, ...
    'bb_padding', BB_PADDING, ...
    'roi_th_size', ROI_TH_SIZE, ...
    'roi_h_side', ROI_H_SIDE, ...
    'hamming_th', HAMMING_TH, ...
    'extract_roi_verbose', EXTRACT_ROI_VERBOSE, ...
    'match_roi_verbose', MATCH_ROI_VERBOSE ...
);

% TRIANGULATION ????
