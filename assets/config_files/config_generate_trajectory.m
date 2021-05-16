%% CONFIGURATION FILE OF ROBOEYE (GENERATE TRAJECTORY)

home_q = [87 81 99 95 90 0];
current_q = [87 81 99 95 90 0];
delta_t = 30;
% fn_cam2robot_coords = @(t) [t(2)*10 + 104, -(t(1)-24)*10, t(3)*10-5];
fn_cam2robot_coords = @(t) t*10;
fn_input = @(x) input(x); 

args.braccio_params = [71 125 125 195 0];
args.post_corr = [0 0 0 0 0];
args.z_min = -5; % [mm] in robot frame
args.joint_safety_radius = [0 30 30 30 0]; % [mm] in robot frame
args.box_coords_grasp = [-4 1 20]; % [cm] in vision frame
args.box_coords_grasp_parabola = [-4 1 12.5]; % [cm] in vision frame
args.n_points_parabola = 10;
args.touchdown_verbose = 0;
args.gothere_verbose = 0;
args.parabolic_traj_verbose = 0;
args.verbose = 1;
