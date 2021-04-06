function robot_fsm_interface(port, baud, cam, vision_args, trajectory_planning_args, fn_cam2robot_coords, fn_robot_input)
% ROBOT_FSM_INTERFACE High level interface with the robot FSM on Arduino.
%
%   ROBOT_FSM_INTERFACE(port, baud, cam, vision_args, trajectory_planning_args,
%   fn_cam2robot_coords, fn_robot_input)
%
%   Input arguments:
%   ------------------
%   port: port of the Arduino serial connection, cf. serialport(...)
%   baud: baud rate of the Arduino serial connection, cf. serialport(...)
%   cam: webcam object of the camera, cf. webcam(...)
%   vision_args: struct of vision parameters, cf below
%   trajectory_planning_args: struct of trajectory planning parameters, cf below
%   fn_cam2robot_coords: function to convert points from vision to robot frame 
%   fn_robot_input: function to acquire input, cf. input(...) or cmdBuffer
%   
%   For details regarding vision_args and trajectory_planning_args refers to
%   get_target_from_vision(...) and generate_trajectory(...) respectively.
%
%   NOTE: this function requires the MATLAB Support Package for USB Webcams.
%
%   See also GENERATE_TRAJECTORY, GET_TARGET, GET_TARGET_FROM_VISION

    fprintf('\n-------- FSM Robot Control start --------\n');
    
    % States of the robot FSM
    STATE = {
        'START'
        'NOP'
        'INITIALIZE'
        'READY'
        'LOAD_TRAJECTORY'
        'POINTWISE_TRAJECTORY'
        'KEYPOINTS_TRAJECTORY'
        'RELEASE'
        'ERROR_STATE'
        'END'
    };
    
    % Help messages of the interface commands
    help_nop = strcat( ...
        'Available commands:\n', ...
        '   0: exit\n', ...
        '   1: initialize robot\n' ...
    );
    help_ready = strcat( ...
        'Available commands:\n', ...
        '   0: release robot\n', ...
        '   1: back to home\n', ...
        '   2: back to home (force)\n', ...
        '   3: move joints\n', ...
        '   4: move to n points\n', ...
        '   5: move to target (pointwise trajectory)\n', ...
        '   6: move to target (keypoints trajectory)\n', ...
        '   7: pick/place object (keypoints trajectory)\n', ...
        '   8: pick/place object (keypoints trajectory + parabola)\n' ...
    );
    help_error_state = strcat( ...
        'Oh no!!!!!!\n', ...
        '   0: release robot\n' ...
    );

    % Validation functions of the interface commands
    fn_val_nop = @(x) isscalar(x) && ismember(x,[0,1]);
    fn_val_ready = @(x) isscalar(x) && ismember(x,[0,1,2,3,4,5,6,7,8]);
    fn_val_error_state = @(x) isscalar(x) && x == 0;
    
    % Time-steps trajectories
    DELTA_T_POINTWISE_TRAJECTORY = 50;
    DELTA_T_KEYPOINTS_TRAJECTORY = 30;
    
    fprintf('\nTransition --> %s\n', STATE{1});

    % Connection to the serial port
    fprintf('Initializing serial connection\n');
    s = serialport(port, baud);
    
    % Read low-level settings of the robot
    fprintf('Receiving robot informations\n');
    QNUM = read(s,1,'uint8');
    fprintf('   QNUM = %d\n', QNUM);
    MAXPOINTS = read(s,1,'uint8');
    fprintf('   MAXPOINTS = %d\n', MAXPOINTS);
    home_position = read(s,QNUM,'uint8');
    fprintf('   home_position = %s\n', mat2str(home_position));
    DELTA_T_START = read(s,1,'uint8') + 1;
    fprintf('   DELTA_T_START = %d [s]\n', DELTA_T_START);
    DELTA_T_INITIALIZE = read(s,1,'uint8') + 1;
    fprintf('   DELTA_T_INITIALIZE = %d [s]\n', DELTA_T_INITIALIZE);
    DELTA_T_RELEASE = read(s,1,'uint8') + 1;
    fprintf('   DELTA_T_RELEASE = %d [s]\n', DELTA_T_RELEASE);
    
    current_position = home_position;
    
    fprintf('Waiting...');
    print_countdown(DELTA_T_START);
    
    % Start robot interface
    exit = 0;
    while ~exit

        % Get the last state transition
        state_tm1_code = uint8(read(s,1,'uint8'));
        state_code = uint8(read(s,1,'uint8'));
        state = STATE{state_code+1};
        fprintf('\nTransition %s --> %s\n', STATE{state_tm1_code+1}, state);
        
        cmd_err = 0;
        switch state
            
            case STATE{1} % START
                % nothing to do here

            case STATE{2} % NOP
                cmd = cmd_acquire(help_nop, fn_val_nop, fn_robot_input);
                cmd_err = cmd_execute(s, cmd);
                
            case STATE{3} % INITIALIZE
                fprintf('Initializing robot\n');
                fprintf('Waiting...');
                print_countdown(DELTA_T_INITIALIZE); 
            
            case STATE{4} % READY
                % Update current position of the robot
                current_position = uint8(read(s,QNUM,'uint8'));
                fprintf('Position: %s\n', mat2str(current_position));
                
                confirm = 0;
                while ~confirm
                
                    % Choose the task to be carried out
                    cmd = cmd_acquire(help_ready, fn_val_ready, fn_robot_input);
                    switch cmd
                        
                        case 0 % release
                            trajectory = [];
                            data_tx = [];
                            confirm = 1;

                        case 1 % back home
                            trajectory = fix_target_q(home_position, current_position);
                            time_trajectory = get_time_trajectory('keypoints', trajectory, current_position, DELTA_T_KEYPOINTS_TRAJECTORY);
                            data_tx = trajectory2serialdata('keypoints', DELTA_T_KEYPOINTS_TRAJECTORY, trajectory);
                            confirm = 1;
                            
                        case 2 % back-home (force)
                            trajectory = [];
                            data_tx = [];
                            confirm = 1;
                        
                        case 3 % move joints
                            [trajectory, time_trajectory, confirm] = generate_trajectory( ...
                                'move-q', current_position, DELTA_T_KEYPOINTS_TRAJECTORY, ...
                                [], [], trajectory_planning_args, [], fn_robot_input ...
                            );
                            data_tx = trajectory2serialdata('keypoints', DELTA_T_KEYPOINTS_TRAJECTORY, trajectory);
                            fprintf('\n');
                        
                        case 4 % move to n points
                            [trajectory, time_trajectory, confirm] = generate_trajectory( ...
                                'move-t-npoints', current_position, DELTA_T_KEYPOINTS_TRAJECTORY, ...
                                [], [], trajectory_planning_args, fn_cam2robot_coords, fn_robot_input ...
                            );
                            data_tx = trajectory2serialdata('keypoints', DELTA_T_KEYPOINTS_TRAJECTORY, trajectory);
                            fprintf('\n');

                        case 5 % move to target (pointwise trajectory)
                            if ~all(current_position == home_position)
                                fprintf('ERROR: robot must be in home position!!\n');
                                trajectory = [];
                                data_tx = [];
                                confirm = 0;
                            else
                                [trajectory, time_trajectory, confirm] = generate_trajectory( ...
                                    'move-t-pointwise', [], DELTA_T_POINTWISE_TRAJECTORY, ...
                                    cam, vision_args, trajectory_planning_args, fn_cam2robot_coords, fn_robot_input ...
                                );
                                data_tx = trajectory2serialdata('pointwise', DELTA_T_POINTWISE_TRAJECTORY, trajectory);
                            end
                            fprintf('\n');

                        case 6 % move to target (keypoints trajectory)
                            [trajectory, time_trajectory, confirm] = generate_trajectory( ...
                                'move-t', current_position, DELTA_T_KEYPOINTS_TRAJECTORY, ...
                                cam, vision_args, trajectory_planning_args, fn_cam2robot_coords, fn_robot_input ...
                            );
                            data_tx = trajectory2serialdata('keypoints', DELTA_T_KEYPOINTS_TRAJECTORY, trajectory);
                            fprintf('\n');
                        
                        case 7 % pick/place object (keypoints trajectory)
                            [trajectory, time_trajectory, confirm] = generate_trajectory( ...
                                'grasp', current_position, DELTA_T_KEYPOINTS_TRAJECTORY, ...
                                cam, vision_args, trajectory_planning_args, fn_cam2robot_coords, fn_robot_input ...
                            );
                            data_tx = trajectory2serialdata('keypoints', DELTA_T_KEYPOINTS_TRAJECTORY, trajectory);
                            fprintf('\n');
                        
                        case 8 % pick/place object (keypoints trajectory + parabola)
                            [trajectory, time_trajectory, confirm] = generate_trajectory( ...
                                'grasp-parabola', current_position, DELTA_T_KEYPOINTS_TRAJECTORY, ...
                                cam, vision_args, trajectory_planning_args, fn_cam2robot_coords, fn_robot_input ...
                            );
                            data_tx = trajectory2serialdata('keypoints', DELTA_T_KEYPOINTS_TRAJECTORY, trajectory);
                            fprintf('\n');
                            
                    end
                end
                
                % Execute the chosen task
                cmd_err = cmd_execute(s, cmd, data_tx);
                
            case STATE{5} % LOAD_TRAJECTORY
                fprintf('Receiving trajectory from robot\n');
                data_rx = uint8(read(s,numel(data_tx),'uint8'));

                % Check the correctness of the data sent to Arduino
                if isequal(data_rx,data_tx)
                   fprintf('Check data PASSED\n');
                   check_trajectory = 1;
                else
                   fprintf('Check data FAILED!!!\n'); 
                   check_trajectory = 0;
                end
                
            case STATE{6} % POINTWISE_TRAJECTORY
                cmd_err = cmd_execute(s, check_trajectory, [], 'ACK data check\n', 'No ACK data check!!!\n');
                
                % Execute the loaded pointwise trajectory
                if check_trajectory && ~cmd_err
                    fprintf('Executing pointwise trajectory\n');
                    fprintf('Waiting... '); 
                    print_countdown(time_trajectory);
                else
                    fprintf('Aborting trajectory\n');
                end
                
            case STATE{7} % KEYPOINTS_TRAJECTORY
                cmd_err = cmd_execute(s, check_trajectory, [], 'ACK data check\n', 'No ACK data check!!!\n');
                
                % Execute the loaded keypoints trajectory
                if check_trajectory && ~cmd_err
                    fprintf('Executing keypoints trajectory\n');
                    fprintf('Waiting...');
                    print_countdown(time_trajectory); 
                else
                    fprintf('Aborting trajectory\n');
                end
                
            case STATE{8} % RELEASE
                fprintf('Releasing robot\n');
                fprintf('Waiting...');
                print_countdown(DELTA_T_RELEASE); 
                
            case STATE{9} % ERROR
                cmd = cmd_acquire(help_error_state, fn_val_error_state, fn_robot_input);
                cmd_err = cmd_execute(s, cmd);

            case STATE{10} % END
                fprintf('Closing serial connection\n');
                exit = 1;
                
        end
        
        % Catch robot ACK failures
        if cmd_err
            exit = 1;
        end

    end
    
    % Disconnection from the serial port
    delete(s);
    clear('s');

    fprintf('\n-------- FSM Robot Control exit ---------\n');
   
end
