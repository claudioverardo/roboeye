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
    
    STATE = {
        'START'
        'NOP'
        'INITIALIZE'
        'READY'
        'LOAD_TRAJECTORY'
        'FOLLOW_TRAJECTORY'
        'BUILT_IN_TRAJECTORY'
        'RELEASE'
        'ERROR_STATE'
        'END'
    };

    DELTA_T_START = 2;
    DELTA_T_INITIALIZE = 7;
    DELTA_T_READ_TRAJECTORY = 1;
    DELTA_T_EXECUTE_LOADED_TRAJECTORY = 10;
    DELTA_T_EXECUTE_BUILT_IN_TRAJECTORY = 30;
    DELTA_T_RELEASE = 2;

    fn_val_nop = @(x) isscalar(x) && ( x == 0 || x == 1 );
    fn_val_ready = @(x) isscalar(x) && ( x == 0 || x == 1 || x == 2 || x == 3 || x == 4 || x == 5 );
    fn_val_error_state = @(x) isscalar(x) && ( x == 0 );
    
    help_nop = 'Available commands:\n   0: exit\n   1: initialize robot\n';
    help_ready = 'Available commands:\n   0: release robot\n   1: back to home\n   2: back to home (force)\n   3: move to target (custom trajectory)\n   4: move to target (built-in trajectory)\n   5: grasp target (built-in trajectory)\n';
    help_error_state = 'Oh no!!!!!!\n   0: release robot\n';

    % Connection to the serial port
    fprintf('\nInitializing serial connection\n');
    s = serialport(port, baud);
    
    fprintf('Waiting...');
    print_countdown(DELTA_T_START);
    
    home_position = [90, 83, 98, 97, 90, 0];
    current_position = home_position;
    last_position = home_position;
    
    QNUM = 6;
    MAXPOINTS = 170;
    
    exit = 0;
    while ~exit

        state_tm1_code = uint8(read(s,1,'uint8'));
        state_code = uint8(read(s,1,'uint8'));
        
        state = STATE{state_code+1};
        
        fprintf('\nTransition %s --> %s\n', STATE{state_tm1_code+1}, state);
        
        cmd_err = 0;
        switch state
            
            case STATE{1} % START

            case STATE{2} % NOP
                cmd = cmd_acquire(help_nop, fn_val_nop, fn_robot_input);
                cmd_err = cmd_execute(s, cmd);
                
            case STATE{3} % INITIALIZE
                fprintf('Initializing robot\n');
                fprintf('Waiting...');
                print_countdown(DELTA_T_INITIALIZE); 
            
            case STATE{4} % READY
                last_position = current_position;
                current_position = uint8(read(s,QNUM,'uint8'));
                fprintf('Position: %s\n', mat2str(current_position));
                
                confirm = 0;
                while ~confirm
                
                    cmd = cmd_acquire(help_ready, fn_val_ready, fn_robot_input);
                
                    switch cmd

                        % back home
                        case 1
                            target_positions = fix_target_q(home_position, current_position, last_position);
                            data_tx = uint8( [ size(target_positions,1) reshape(target_positions',1,[]) ] );
                            confirm = 1;

                        % move to target (custom trajectory)
                        case 3
                            [trajectory, confirm] = generate_custom_trajectory(cam, vision_args, fn_cam2robot_coords, fn_robot_input);
                            data_tx = uint8(reshape(trajectory',1,[]));
                            fprintf('\n');

                        % move to target (built-in trajectory)
                        case 4
                            [target_positions, confirm] = generate_built_in_trajectory('move', current_position, last_position, cam, vision_args, trajectory_planning_args, [], fn_cam2robot_coords, fn_robot_input);
                            data_tx = uint8( [ size(target_positions,1) reshape(target_positions',1,[]) ] );
                            fprintf('\n');
                        
                        % grasp target (built-in trajectory)
                        case 5
                            [target_positions, confirm] = generate_built_in_trajectory('grasp', current_position, last_position, cam, vision_args, trajectory_planning_args, objects_dict, fn_cam2robot_coords, fn_robot_input);
                            data_tx = uint8( [ size(target_positions,1) reshape(target_positions',1,[]) ] );
                            fprintf('\n');

                        % release / back-home (force)
                        otherwise
                            data_tx = [];
                            confirm = 1;
                    end
                end
                
                cmd_err = cmd_execute(s, cmd, data_tx);
                
            case STATE{5} % LOAD_TRAJECTORY
                
                fprintf('Waiting... '); 
                print_countdown(DELTA_T_READ_TRAJECTORY);
                
                fprintf('Receiving trajectory from robot\n');
                data_rx = uint8(read(s,MAXPOINTS*QNUM,'uint8'));

                if isequal(data_rx,data_tx)
                   fprintf('Check data PASSED\n'); 
                else
                   fprintf('Check data FAILED!!!\n'); 
                end
                
            case STATE{6} % FOLLOW_TRAJECTORY
                fprintf('Executing custom trajectory\n');
                fprintf('Waiting... '); 
                delta_t_custom_trajectory = estimate_time_trajectory('custom', trajectory, DELTA_T_EXECUTE_LOADED_TRAJECTORY);
                print_countdown(delta_t_custom_trajectory);
                
            case STATE{7} % BUILT_IN_TRAJECTORY
                fprintf('Executing built-in trajectory\n');
                fprintf('Waiting...');
                delta_t_built_in_trajectory = estimate_time_trajectory('built-in', target_positions, DELTA_T_EXECUTE_BUILT_IN_TRAJECTORY);
                print_countdown(delta_t_built_in_trajectory); 
                
            case STATE{8} % RELEASE
                fprintf('Releasing robot\n');
                fprintf('Waiting...');
                print_countdown(DELTA_T_RELEASE); 
                
            case STATE{9} % ERROR
                cmd = cmd_acquire(help_error_state, fn_val_error_state, fn_robot_input);
                cmd_err = cmd_execute(s, cmd);

            case STATE{10} % END
                exit = 1;
                
        end
        
        if cmd_err
            exit = 1;
        end

    end

    fprintf('\n-------- FSM Robot Control exit ---------\n');
    
    delete(s);
    clear('s');
   
end
