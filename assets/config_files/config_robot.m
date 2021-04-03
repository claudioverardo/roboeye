%% CONFIGURATION FILE OF ROBOEYE

aruco_markers_file = '../aruco_markers/aruco_markers_7x7.mat';
aruco_real_sides_file = '../aruco_markers/aruco_markers_7x7_real_sides.mat';
K_file = '../calibration/intrinsics_cam1/K.mat';
R_cam_file = '../calibration/extrinsics_cam1_tests_robot/R_cam.mat';
t_cam_file = '../calibration/extrinsics_cam1_tests_robot/t_cam.mat';
intrinsics_file = '../calibration/intrinsics_cam1/intrinsics.mat';

vision_args.aruco_markers = load(aruco_markers_file).aruco_markers;
vision_args.aruco_real_sides = load(aruco_real_sides_file).aruco_real_sides;
vision_args.K = load(K_file, 'K').K';
vision_args.R_cam = load(R_cam_file, 'R_cam').R_cam';
vision_args.t_cam = load(t_cam_file, 't_cam').t_cam';
vision_args.k = load(intrinsics_file, 'intrinsics').intrinsics.radial;

% ROI extraction parameters
vision_args.options.roi_extraction_method = 'adaptth-moore'; % adaptth-moore, canny-dfs, canny-dfs-c
vision_args.options.adaptth_sensitivity = 1;        % [0,1]
vision_args.options.adaptth_statistic = 'gaussian';   % mean, gaussian, median
vision_args.options.adaptth_neighborhood = [135 241]; % default for 1080x1920 -> [135 241]
vision_args.options.canny_th_low = 0.01;
vision_args.options.canny_th_high = 0.10;

% ROI refinement parameters
vision_args.options.roi_refinement_method = 'geometric'; % rdp, geometric
vision_args.options.roi_size_th = 50;
vision_args.options.rdp_th =0.2;  % Ramer–Douglas–Peucker threshold
vision_args.options.roi_sum_angles_tol  = 10; % [degrees]
vision_args.options.roi_parallelism_tol = 10; % [degrees]
vision_args.options.roi_side_th_low = 1/100; % [% diag(img)]
vision_args.options.roi_side_th_high = 1/5;   % [% diag(img)]

% ROI matching parameters
vision_args.options.roi_bb_padding = 2;
vision_args.options.roi_h_side = 80;
vision_args.options.roi_hamming_th = 2;

% Debug/Analysis
vision_args.options.roi_extraction_verbose = 0;
vision_args.options.roi_refinement_verbose = 0;
vision_args.options.roi_matching_verbose = 0;
vision_args.options.roi_pose_estimation_verbose = 2;
vision_args.options.aruco_detection_verbose = 0;
vision_args.options.verbose = 0; % aruco_pose_estimation_verbose

% Dictionary of objects
% offsets in [mm]
objects_dict(1).name = '';
objects_dict(1).offset_h = 0;
objects_dict(1).offset_r = 0;

objects_dict(2).name = '';
objects_dict(2).offset_h = 0;
objects_dict(2).offset_r = 0;

objects_dict(3).name = '';
objects_dict(3).offset_h = 0;
objects_dict(3).offset_r = 0;

objects_dict(4).name = '';
objects_dict(4).offset_h = 0;
objects_dict(4).offset_r = 0;

objects_dict(5).name = 'sharpener';
objects_dict(5).offset_h = -40;
objects_dict(5).offset_r = 20;

objects_dict(6).name = 'tissues';
objects_dict(6).offset_h = -15;
objects_dict(6).offset_r = 20;

objects_dict(7).name = '';
objects_dict(7).offset_h = 0;
objects_dict(7).offset_r = 0;

objects_dict(8).name = '';
objects_dict(8).offset_h = 0;
objects_dict(8).offset_r = 0;

