close all % clear

cam = webcam(1);
% cam = [];

config_file = '../assets/config_files/config_robot';
run(config_file);

% plot_aruco_markers(vision_args.aruco_markers);

% seriallist
baud = 115200;
port = 'COM3';

% function that maps camera coordinates in robot coordinates
cam2robot_coords = @(t) [t(2)*10 + 104, -(t(1)-24)*10, t(3)*10-5];

% function that defines from where to acquire the commands
robot_input = @(x) input(x); % command window

% pick the only one object in the scene
% robot_buffer = cmdBuffer({1 7 1 0 1 1 0 0});
% pick glass -> sharpener -> cup -> candies -> glue -> tissues
% robot_buffer = cmdBuffer({1 7 1 2 3 0 1 7 1 2 2 0 1 7 1 2 1 0 1 7 1 2 5 0 1 7 1 2 4 0 1 7 1 2 6 0 1 1 0 0});
% robot_input = @(x) robot_buffer.getCmd(x); % custom buffer

robot_fsm_interface(port, baud, cam, vision_args, trajectory_planning_args, cam2robot_coords, robot_input)

