close all % clear

img_source = webcam(1);
% 
%% SERIAL PORT
fprintf('---- CONNECTION -----\n');
s = serialport('COM3',115200);
% seriallist
print_countdown(12);
fprintf('DONE\n');

%%
% fprintf('POSE ESTIMATION START IN');
print_countdown(3);

%% ARUCO DETECTION+POSE ESTIMATION
config_file = '../assets/config_files/config_pose_estimation';
K_file = '../assets/calibration/intrinsics_cam1/K.mat';
R_cam_file = '../assets/calibration/extrinsics_cam1_test_robot1/R_cam.mat';
t_cam_file = '../assets/calibration/extrinsics_cam1_test_robot1/t_cam.mat';
aruco_markers_file = '../assets/aruco_markers/aruco_markers_8x8.mat';
aruco_real_sides = [3 3 3 4 4 6]; % [cm]

run(config_file);
load(aruco_markers_file);
load(K_file, 'K');
load(R_cam_file, 'R_cam');
load(t_cam_file, 't_cam');
img = get_image(img_source);

% literature -> Matlab convention
K = K';       
R_cam = R_cam';
t_cam = t_cam';

% Launch Aruco Pose Estimation
[rois, i_arucos, rois_R, rois_t] = aruco_pose_estimation( ...
    img, aruco_markers, aruco_real_sides, K, R_cam, t_cam, ...
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

t = rois_t{1};
x_robot = t(2)*10 + 104;
y_robot = - t(1)*10;

%%
% fprintf('TRAJECTORY START IN');
% print_countdown(5);

%% TREJECTORY PLANNING

[trajectory, check_trajectory] = touchdown(x_robot, y_robot, 0);

if (check_trajectory == 1)
    error('ERROR: TRAJECTORY NOT VALID')
end

%%
% fprintf('CONTROL START IN');
% print_countdown(5);

%% ROBOT CONTROL
q_trajectory = uint8(trajectory);
MAXPOINTS = size(q_trajectory,1);
QNUM = size(q_trajectory,2);

M_TX = reshape(q_trajectory.',1,[]);
M_RX = zeros(size(M_TX));

fprintf('-------- TX --------\n');
write(s, M_TX, 'uint8')
fprintf('DONE\n');
pause(2);

fprintf('-------- RX --------\n');
M_RX = uint8(read(s,MAXPOINTS*QNUM,'uint8'));
fprintf('DONE\n');           

if isequal(M_RX,M_TX)
   fprintf('DATA OK\n'); 
else
   fprintf('DATA ERROR!\n'); 
end

fprintf('START TRAJECTORY\n');

delete(s);
