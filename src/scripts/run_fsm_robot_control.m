close all % clear

cam = webcam(1);
config_file = '../assets/config_files/config_robot';
run(config_file);

% seriallist
baud = 115200;
port = 'COM3';

% function that maps camera coordinates in robot coordinates
cam2robot_coords = @(t) [t(2)*10 + 104, -t(1)*10, 0];

% function that defines from whete to acquire the commands
% robot_input = @(x) input(x); % command window

robot_buffer = cmdBuffer([1 2 1 1 2 1 2 1 1 2 1 2 1 1 2 1 0 0]);
robot_input = @(x) robot_buffer.getCmd(x); % custom buffer

fsm_robot_control(port, baud, cam, vision_args, cam2robot_coords, robot_input)


