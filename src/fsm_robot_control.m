function fsm_robot_control(port, baud, cam, vision_args, fn_cam2robot_coords, fn_robot_input)

    fprintf('\n-------- FSM Robot Control start --------\n');

    STATE = {
        'START'
        'NOP'
        'INITIALIZE'
        'HOME'
        'LOAD_TRAJECTORY'
        'FOLLOW_TRAJECTORY'
        'DONE'
        'BACK_HOME'
        'RELEASE'
        'ERROR'
        'END'
    };

    ACK = 6;

    DELTA_T_START = 3;
    DELTA_T_INITIALIZE = 7;
    DELTA_T_READ_TRAJECTORY = 2;
    DELTA_T_BACK_HOME = 4;
    DELTA_T_RELEASE = 3;

    fn_val_nop = @(x) x == 0 || x == 1;
    fn_val_home = @(x) x == 0 || x == 1 || x == 2;
    fn_val_load_trajectory = @(x) x == 1 || x == 2;
    fn_val_follow_trajectory = @(x) x == 0 || x == 1 || x == 2;
    fn_val_done = @(x) x == 0 || x == 1;
    fn_val_error_state = @(x) x == 0;
    
    help_nop = 'Available commands:\n   0: exit\n   1: initialize robot\n';
    help_home = 'Available commands:\n   0: release robot\n   1: adjust home\n   2: load trajectory\n';
    help_load_trajectory = 'Available commands:\n   1: target from camera\n   2: target from user\n';
    help_follow_trajectory = 'Available commands:\n   0: release robot\n   1: back to home\n   2: execute trajectory\n';
    help_done = 'Available commands:\n   0: release robot\n   1: back to home\n';
    help_error_state = 'Oh no!!!!!!\n   0: release robot\n';

    % Connection to the serial port
    fprintf('\nInitializing serial connection\n');
    s = serialport(port, baud);
    
    fprintf('Waiting...');
    print_countdown(DELTA_T_START);
    
    exit = 0;
    while ~exit

        state_tm1_code = uint8(read(s,1,'uint8'));
        state_code = uint8(read(s,1,'uint8'));
        
        state = STATE{state_code+1};
        
        fprintf('\nTransition %s --> %s\n', STATE{state_tm1_code+1}, state);

        switch state
            
            case STATE{1} % START

            case STATE{2} % NOP
                cmd = acquire_execute_cmd(s, help_nop, fn_val_nop, fn_robot_input);
                
            case STATE{3} % INITIALIZE
                fprintf('Initializing robot\n');
                fprintf('Waiting...');
                print_countdown(DELTA_T_INITIALIZE); 
            
            case STATE{4} % HOME
                cmd = acquire_execute_cmd(s, help_home, fn_val_home, fn_robot_input);
                
            case STATE{5} % LOAD_TRAJECTORY
                
                invalid_trajectory = 1;
                while invalid_trajectory
                
                    cmd = acquire_cmd(help_load_trajectory, fn_val_load_trajectory, fn_robot_input);
                    switch cmd

                        case 1
                            fprintf('   Acquiring image\n');
                            img = snapshot(cam);

                            fprintf('   Executing Aruco pose estimation\n');
                            [rois, i_arucos, rois_R, rois_t] = aruco_pose_estimation( ...
                                img, vision_args.aruco_markers, vision_args.aruco_real_sides, ...
                                vision_args.K, vision_args.R_cam, vision_args.t_cam, vision_args.k, ...
                                vision_args.options ...
                            );
                        
                            %fprintf('\n');
                            n_rois = length(rois);
                            for i=1:n_rois
                                fprintf('      ROI %d -- Aruco %d -- X = %g  Y = %g  Z = %g [cm]\n', i, i_arucos(i), rois_t{i}(1), rois_t{i}(2), rois_t{i}(3));
                            end
                        
                            idx = fn_robot_input(sprintf('   Choose ROI target (1-%d): ', n_rois));
                            t = rois_t{idx};

                        case 2
                            t = fn_robot_input('   Insert target\n   [X,Y,Z] = ');

                    end

                    fprintf('   Target (camera frame): X = %g  Y = %g  Z = %g [cm]\n', t(1), t(2), t(3));
                    
                    t_robot = fn_cam2robot_coords(t);
                    x_robot = t_robot(1);
                    y_robot = t_robot(2);
                    z_robot = t_robot(3);
                    
                    fprintf('   Target (robot frame):  X = %g  Y = %g  Z = %g [mm]\n', t_robot(1), t_robot(2), t_robot(3));

                    fprintf('   Computing trajectory\n');
                    [trajectory, invalid_trajectory] = touchdown(x_robot, y_robot, z_robot);

                    if invalid_trajectory
                        fprintf('   ERROR: trajectory not valid!\n\n');
                    end
                    
                end
                
                fprintf('   Sending trajectory to robot\n');
                
                q_trajectory = uint8(trajectory);
                MAXPOINTS = size(q_trajectory,1);
                QNUM = size(q_trajectory,2);

                M_TX = reshape(q_trajectory.',1,[]);

                write(s, M_TX, 'uint8')
                fprintf('   Waiting... '); 
                print_countdown(DELTA_T_READ_TRAJECTORY);
                
                fprintf('   Receiving trajectory from robot\n');
                M_RX = uint8(read(s,MAXPOINTS*QNUM,'uint8'));

                if isequal(M_RX,M_TX)
                   fprintf('   Check data PASSED\n'); 
                else
                   fprintf('   Check data FAILED!!!\n'); 
                end
                
            case STATE{6} % FOLLOW_TRAJECTORY
                cmd = acquire_execute_cmd(s, help_follow_trajectory, fn_val_follow_trajectory, fn_robot_input);
                
            case STATE{7} % DONE
                cmd = acquire_execute_cmd(s, help_done, fn_val_done, fn_robot_input);
                
            case STATE{8} % BACK_HOME
                fprintf('Back to home position\n');
                fprintf('Waiting...');
                print_countdown(DELTA_T_BACK_HOME); 
                
            case STATE{9} % RELEASE
                fprintf('Releasing robot\n');
                fprintf('Waiting...');
                print_countdown(DELTA_T_RELEASE); 
                
            case STATE{10} % ERROR
                cmd = acquire_execute_cmd(s, help_error_state, fn_val_error_state, fn_robot_input);

            case STATE{11} % END
                exit = 1;
                
        end

    end

    fprintf('\n-------- FSM Robot Control exit ---------\n');
    
    delete(s);
    clear('s');
   
end

function cmd = acquire_cmd(help, fn_val, fn_robot_input)
   
    fprintf(help);
    
    check_cmd_sw = 0;
    while ~check_cmd_sw
        cmd = fn_robot_input('Command: ');
        check_cmd_sw = fn_val(cmd);
        if ~check_cmd_sw
            fprintf('Command not valid\n');
        end
    end  
    
end

function cmd = acquire_execute_cmd(s, help, fn_val, fn_robot_input)

    ACK = 6;
    
    check_cmd_hw = 0;
    while ~check_cmd_hw
        cmd = acquire_cmd(help, fn_val, fn_robot_input);
        write(s, uint8(cmd), 'uint8');
        cmd_robot = uint8(read(s,1,'uint8'));
        check_cmd_hw = cmd_robot == ACK;
        if ~check_cmd_hw
            fprintf('No ACK from the robot!!!\n');
        else 
            fprintf('ACK received\n');
        end
    end        

end
