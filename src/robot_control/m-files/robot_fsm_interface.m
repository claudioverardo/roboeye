function robot_fsm_interface(port, baud, cam, vision_args, trajectory_planning_args, objects_dict, fn_cam2robot_coords, fn_robot_input)
% ROBOT_FSM_INTERFACE TODO
%
%   ROBOT_FSM_INTERFACE(port, baud, cam, vision_args, fn_cam2robot_coords, fn_robot_input)
%
%   Input arguments:
%   ------------------
%   port:
%   baud:
%   cam: 
%   vision_args:
%   fn_cam2robot_coords: 
%   fn_robot_input: 
%
%   See also TODO

    fprintf('\n-------- FSM Robot Control start --------\n');
    
    % States of the robot FSM
    STATE = {
        'START'
        'NOP'
        'INITIALIZE'
        'READY'
        'LOAD_TRAJECTORY'
        'CUSTOM_TRAJECTORY'
        'KEYPOINTS_TRAJECTORY'
        'RELEASE'
        'ERROR_STATE'
        'END'
    };
    
    % Help messages of the interface commands
    help_nop = 'Available commands:\n   0: exit\n   1: initialize robot\n';
    help_ready = 'Available commands:\n   0: release robot\n   1: back to home\n   2: back to home (force)\n   3: move joints\n   4: move to n points\n   5: move to target (custom trajectory)\n   6: move to target (keypoints trajectory)\n   7: pick/place object (keypoints trajectory)\n   8: pick/place object (keypoints trajectory + parabola)\n';
    help_error_state = 'Oh no!!!!!!\n   0: release robot\n';

    % Validation functions of the interface commands
    fn_val_nop = @(x) isscalar(x) && ( x == 0 || x == 1 );
    fn_val_ready = @(x) isscalar(x) && ( x == 0 || x == 1 || x == 2 || x == 3 || x == 4 || x == 5 || x == 6 || x == 7 || x == 8 );
    fn_val_error_state = @(x) isscalar(x) && ( x == 0 );
    
    % Time-steps trajectories
    DELTA_T_CUSTOM_TRAJECTORY = 50;
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
    last_position = home_position;
    
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
                last_position = current_position;
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
                            trajectory = fix_target_q(home_position, current_position, last_position);
                            data_tx = trajectory2serialdata('keypoints', DELTA_T_KEYPOINTS_TRAJECTORY, trajectory);
                            confirm = 1;
                            
                        case 2 % back-home (force)
                            trajectory = [];
                            data_tx = [];
                            confirm = 1;
                        
                        case 3 % move joints
                            [trajectory, confirm] = generate_keypoints_trajectory('move-q', current_position, last_position, cam, vision_args, trajectory_planning_args, [], fn_cam2robot_coords, fn_robot_input);
                            data_tx = trajectory2serialdata('keypoints', DELTA_T_KEYPOINTS_TRAJECTORY, trajectory);
                            fprintf('\n');
                        
                        case 4 % move to n points
                            [trajectory, confirm] = generate_keypoints_trajectory('move-n', current_position, last_position, cam, vision_args, trajectory_planning_args, [], fn_cam2robot_coords, fn_robot_input);
                            data_tx = trajectory2serialdata('keypoints', DELTA_T_KEYPOINTS_TRAJECTORY, trajectory);
                            fprintf('\n');

                        case 5 % move to target (custom trajectory)
                            [trajectory, confirm] = generate_custom_trajectory(current_position, home_position, cam, vision_args, fn_cam2robot_coords, fn_robot_input);
                            data_tx = trajectory2serialdata('custom', DELTA_T_CUSTOM_TRAJECTORY, trajectory);
                            fprintf('\n');

                        case 6 % move to target (keypoints trajectory)
                            [trajectory, confirm] = generate_keypoints_trajectory('move', current_position, last_position, cam, vision_args, trajectory_planning_args, [], fn_cam2robot_coords, fn_robot_input);
                            data_tx = trajectory2serialdata('keypoints', DELTA_T_KEYPOINTS_TRAJECTORY, trajectory);
                            fprintf('\n');
                        
                        case 7 % pick/place object (keypoints trajectory)
                            [trajectory, confirm] = generate_keypoints_trajectory('grasp', current_position, last_position, cam, vision_args, trajectory_planning_args, objects_dict, fn_cam2robot_coords, fn_robot_input);
                            data_tx = trajectory2serialdata('keypoints', DELTA_T_KEYPOINTS_TRAJECTORY, trajectory);
                            fprintf('\n');
                        
                        case 8 % pick/place object (keypoints trajectory + parabola)
                            [trajectory, confirm] = generate_keypoints_trajectory('grasp-parabola', current_position, last_position, cam, vision_args, trajectory_planning_args, objects_dict, fn_cam2robot_coords, fn_robot_input);
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
                
            case STATE{6} % CUSTOM_TRAJECTORY
                cmd_err = cmd_execute(s, check_trajectory, [], 'ACK data check received\n', 'ACK data check not received!!!\n');
                
                % Execute the loaded custom trajectory
                if check_trajectory && ~cmd_err
                    fprintf('Executing custom trajectory\n');
                    fprintf('Waiting... '); 
                    delta_t_custom_trajectory = estimate_time_trajectory('custom', trajectory, DELTA_T_CUSTOM_TRAJECTORY);
                    print_countdown(delta_t_custom_trajectory);
                else
                    fprintf('Aborting trajectory\n');
                end
                
            case STATE{7} % KEYPOINTS_TRAJECTORY
                cmd_err = cmd_execute(s, check_trajectory, [], 'ACK data check received\n', 'ACK data check not received!!!\n');
                
                % Execute the loaded keypoints trajectory
                if check_trajectory && ~cmd_err
                    fprintf('Executing keypoints trajectory\n');
                    fprintf('Waiting...');
                    delta_t_keypoints_trajectory = estimate_time_trajectory('keypoints', trajectory, DELTA_T_KEYPOINTS_TRAJECTORY);
                    print_countdown(delta_t_keypoints_trajectory); 
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
