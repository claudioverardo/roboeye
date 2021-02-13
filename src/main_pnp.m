close all
clear

% ROIs extraction parameters
CANNY_TH_LOW  = 0.01;
CANNY_TH_HIGH = 0.10;
RDP_TH        = 0.1; % Ramer–Douglas–Peucker

% Aruco matching parameters
BB_PADDING  = 2;
ROI_TH_SIZE = 0.5;
ROI_H_SIDE  = 40;
HAMMING_TH  = 1;

% Debug
EXTRACT_ROI_VERBOSE = 0;
MATCH_ROI_VERBOSE   = 1;
PNP_ROI_VERBOSE     = 1;

% Real world length of the aruco
aruco_real_side = 0.06; % [m]
roi_world = aruco_real_side * [
    %  x    y    z
    -0.5  0.5    0
     0.5  0.5    0
     0.5 -0.5    0
    -0.5 -0.5    0
];

% Load image
% img = imread('../assets/img_tests/test_iPhone/File_001.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_002.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_003.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_004.jpeg');
img = imread('../assets/img_tests/test_iPhone/File_005.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_006.jpeg');
% img = imread('../assets/img_tests/test_iPhone/File_007.jpeg');
% img = imresize(img, [900 1200]);

% Load markers
load('aruco_markers_8x8.mat');

% Load camera intrinsics
load('data/K_iPhone.mat');

fprintf('------- Aruco Detection -------\n');
[rois_found, i_rois, i_arucos, k_rots] = aruco_detection(...
    img, aruco_markers, ...
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

roi = rois_found(:,:,1);

fprintf('------- Aruco PnP -------\n');
[R, t] = aruco_pnp( ...
    roi, roi_world, K, ...
    'verbose', PNP_ROI_VERBOSE ...
);


centroid_world = mean(roi_world);

P = K*[R t];
centroid_proj = htx(P, centroid_world')';
roi_proj = htx(P, roi_world')';

axes_world = 1.5*aruco_real_side * [
    1 0 0
    0 1 0
    0 0 1
];
axes_proj = htx(P, axes_world')';

figure;
imshow(img);
hold on;
line([roi(:,1); roi(1,1)], ...
     [roi(:,2); roi(1,2)], ...
     'color','r','linestyle','-','linewidth',1.5, ...
     'marker','o','markersize',5);
plot(roi(1,1),roi(1,2), 'go', 'MarkerFaceColor', 'g');
plot(centroid_proj(1),centroid_proj(2),'go');
% plot(roi_proj(:,1),roi_proj(:,2),'go');

color = ['r', 'g', 'b'];
for i=1:3
    line([centroid_proj(1,1) axes_proj(i,1)], ...
         [centroid_proj(1,2) axes_proj(i,2)], ...
         'color',color(i),'linestyle','-','linewidth',1.5, ...
         'marker','o','markersize',5);
end



%% --------------------------
% RISULTATI ARUCO DETECTION 1
% img = imread('../assets/img_tests/test_iPhone/File_006.jpeg');
% k_rot = 1;
% roi_nosort = [
%     2231  731
%     2375  797
%     2257  896
%     2117  823
% ];
% %% --------------------------
% % RISULTATI ARUCO DETECTION 2
% img = imread('../assets/img_tests/test_iPhone/File_003.jpeg');
% k_rot = 1;
% roi_nosort = [
%     2202 1300
%     2479 1350
%     2421 1596
%     2133 1535
% ];
% %% --------------------------
% img = imread('../assets/img_tests/test_iPhone/File_002.jpeg');
% k_rot = 2;
% roi_nosort = [
%     2344  562
%     2456  826
%     2082  894
%     1993  630
% ];
% %% --------------------------
