close all % clear

%% ROBOT-USER INTERFACE
%--------------------------------------------------------------------------

% Configuration parameters of the Arduino serial connection
baud = 115200;
port = 'COM3';

% Camera used by the vision module
% (if empty the target from vision feature is disabled)
cam = webcam(1);
% cam = [];

% Configuration parameters of vision and trajectory planning modules
config_file = '../assets/config_files/config_robot';
run(config_file);

% Function that maps vision coordinates into robot coordinates
cam2robot_coords = @(t) [t(2)*10 + 104, -(t(1)-24)*10, t(3)*10-5];

% Function that defines from where to acquire the commands
% (the default command window of Matlab or a pre-filled buffer)

% Command window
robot_input = @(x) input(x); 

% Custom buffer: pick the only one object in the scene
% robot_buffer = cmdBuffer({1 7 1 0 1 1 0 0});
% robot_input = @(x) robot_buffer.getCmd(x);

% Custom buffer: pick glass (ID=3) > sharpener (ID=2) > cup (ID=1) > candies (ID=5) > glue (ID=4) > tissues (ID=6)
% robot_buffer = cmdBuffer({1 7 1 2 3 0 1 7 1 2 2 0 1 7 1 2 1 0 1 7 1 2 5 0 1 7 1 2 4 0 1 7 1 2 6 0 1 1 0 0});
% robot_input = @(x) robot_buffer.getCmd(x);

% Launch the Matlab interface with the Arduino FSM
robot_fsm_interface(port, baud, cam, vision_args, trajectory_planning_args, cam2robot_coords, robot_input)

