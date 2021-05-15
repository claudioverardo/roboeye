
home_q = [87 81 99 95 90 0];
current_q = [87 81 99 95 90 0];
delta_t = 30;
fn_cam2robot_coords = @(t) [t(2)*10 + 104, -(t(1)-24)*10, t(3)*10-5];
% fn_cam2robot_coords = @(t) t;
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

% Choose the method to generate the trajectory
help_msg = strcat( ...
    'Available trajectories:\n', ...
    '   1: back to home\n', ...
    '   3: move joints\n', ...
    '   4: move to n points\n', ...
    '   5: move to target (pointwise trajectory)\n', ...
    '   6: move to target (keypoints trajectory)\n', ...
    '   7: pick/place object (keypoints trajectory)\n', ...
    '   8: pick/place object (keypoints trajectory + parabola)\n' ...
);
fn_val = @(x) isscalar(x) && ismember(x,[1,3,4,5,6,7,8]);
cmd = cmd_acquire(help_msg, fn_val, fn_input);

switch cmd
    case 1; method = 'move-q';
    case 3; method = 'move-t-npoints';
    case 4; method = 'move-t-npoints';
    case 5; method = 'move-t-pointwise';
    case 6; method = 'move-t';
    case 7; method = 'grasp';
    case 8; method = 'grasp-parabola';
end

% Launch the trajectory generator
[trajectory, time_trajectory] = generate_trajectory( ...
    method, home_q, current_q, delta_t, ...
    [], [], args, fn_cam2robot_coords, fn_input ...
);