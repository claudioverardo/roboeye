close all; % clear

%% GENERATE TRAJECTORY TOOL
%--------------------------------------------------------------------------

config_file = '../assets/config_files/config_generate_trajectory';
run(config_file);

fprintf('   Home position: %s\n', mat2str(home_q));
fprintf('Current position: %s\n', mat2str(current_q));

% Choose the method to generate the trajectory
help_msg = strcat( ...
    '\nAvailable trajectories:\n', ...
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
    case 1; method = 'back-home';
    case 3; method = 'move-q';
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