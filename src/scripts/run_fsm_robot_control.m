close all % clear

cam = webcam(1);
config_file = '../assets/config_files/config_robot';
run(config_file);

% seriallist
baud = 115200;
port = 'COM3';

% function that maps camera coordinates in robot coordinates
cam2robot_coords = @(t) [t(2)*10 + 104, -t(1)*10, 0];


fsm_robot_control(port, baud, cam, vision_args, cam2robot_coords)
