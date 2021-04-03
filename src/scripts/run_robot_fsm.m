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
% cam2robot_coords = @(t) [t(2)*10 + 104, -(t(1)-3)*10, 0];
% cam2robot_coords = @(t) [t(2)*10 + 104, -(t(1)-24)*10, max((t(3)+1)*10-5,0.5)];
% cam2robot_coords = @(t) [t(2)*10 + 104, -(t(1)-24)*10, max((t(3)-3)*10-5,0.5)];
cam2robot_coords = @(t) [t(2)*10 + 104, -(t(1)-24)*10, max(t(3)*10-5,0)];

% function that defines from whete to acquire the commands
robot_input = @(x) input(x); % command window

% robot_buffer = cmdBuffer({1 3 2 [18 10 0] 2 1 0 0});
% robot_input = @(x) robot_buffer.getCmd(x); % custom buffer

robot_fsm_interface(port, baud, cam, vision_args, objects_dict, cam2robot_coords, robot_input)

