% CONFIGURATION FILE OF ROBOEYE

%% ROBOT VISION
%--------------------------------------------------------------------------
vision_args.aruco_markers = load('../aruco_markers/aruco_markers_7x7.mat').aruco_markers;
vision_args.aruco_real_sides = load('../aruco_markers/aruco_markers_7x7_real_sides.mat').aruco_real_sides;
vision_args.K = load('../calibration/intrinsics_cam1/K.mat').K';
vision_args.R_cam = load('../calibration/extrinsics_cam1_tests_robot/R_cam.mat').R_cam';
vision_args.t_cam = load('../calibration/extrinsics_cam1_tests_robot/t_cam.mat').t_cam';
vision_args.k = load('../calibration/intrinsics_cam1/intrinsics.mat').intrinsics.radial;
vision_args.options = struct();

% ROI extraction parameters
vision_args.options.roi_extraction_method = 'adaptth-moore'; % adaptth-moore, canny-dfs, canny-dfs-c
vision_args.options.adaptth_sensitivity = 0.7;        % [0,1]
vision_args.options.adaptth_statistic = 'gaussian';   % mean, gaussian, median
vision_args.options.adaptth_neighborhood = [135 241]; % default for 1080x1920 -> [135 241]
vision_args.options.canny_th_low = 0.01;
vision_args.options.canny_th_high = 0.10;

% ROI refinement parameters
vision_args.options.roi_refinement_method = 'geometric'; % rdp, geometric
vision_args.options.roi_size_th = 50;
vision_args.options.rdp_th = 0.2;  % Ramer–Douglas–Peucker threshold
vision_args.options.roi_sum_angles_tol  = 10; % [degrees]
vision_args.options.roi_parallelism_tol = 10; % [degrees]
vision_args.options.roi_side_th_low = 1/100;  % [% diag(img)]
vision_args.options.roi_side_th_high = 1/5;   % [% diag(img)]
vision_args.options.roi_angle_th_low = 20;    % [degrees]
vision_args.options.roi_angle_th_high = 160;  % [degrees]

% ROI matching parameters
vision_args.options.roi_bb_padding = 2;
vision_args.options.roi_h_side = 80;
vision_args.options.roi_hamming_th = 2;

% Debug/Analysis
vision_args.options.roi_extraction_verbose = 0;
vision_args.options.roi_refinement_verbose = 0;
vision_args.options.roi_matching_verbose = 0;
vision_args.options.roi_pose_estimation_verbose = 0;
vision_args.options.aruco_detection_verbose = 0;
vision_args.options.verbose = 0; % aruco_pose_estimation_verbose


%% ROBOT TRAJECTORY PLANNING
%--------------------------------------------------------------------------
trajectory_planning_args.braccio_params = [71 125 125 195 0];
trajectory_planning_args.post_corr = [-3 -1 -1 4 0];
trajectory_planning_args.z_min = 0; % [mm] in robot frame
trajectory_planning_args.box_coords_grasp = [-4 1 20]; % [cm] in vision frame
trajectory_planning_args.box_coords_grasp_parabola = [-4 1 12.5]; % [cm] in vision frame
trajectory_planning_args.n_points_parabola = 10;
trajectory_planning_args.touchdown_verbose = 0;
trajectory_planning_args.gothere_verbose = 0;
trajectory_planning_args.parabolic_traj_verbose = 0;
trajectory_planning_args.objects_dict = load('../objects_dict/objects_dict_7x7.mat').objects_dict;

% trajectory_planning_args.box_coords_grasp = [20 117 0 122 90 73];
% trajectory_planning_args.box_coords_grasp_parabola = [-1 -1 12.5]; % [17.6 107 0.863 133 90 73]
% trajectory_planning_args.box_coords_grasp_parabola = [-1 -4 12.5]; % [11.4 110 0.637 137 90 73]
