function [t_robot_q_fixed, t_robot_q, confirm] = generate_built_in_trajectory(current_q, last_q, cam, vision_args, fn_cam2robot_coords, fn_robot_input)

    QNUM = numel(current_q);
    
    invalid_trajectory = 1;
    while invalid_trajectory
        
        fprintf('\n----- Generate built-in trajectory -----\n');

        cmd = cmd_acquire( ...
            'Available commands:\n   1: target from camera\n   2: target from user\n', ...
            @(x) isscalar(x) && ( x == 1 || x == 2 ), ...
            fn_robot_input ...
        );
    
        switch cmd
            
            case 1
                [t, R, i_aruco] = target_from_vision(cam, vision_args, fn_robot_input);
            
            case 2
        
                t = cmd_acquire( ...
                    '   Insert target position [M1,M2,M3,M4,M5,M6] or [X,Y,Z]\n', ...
                    @(x) isvector(x) && (numel(x)==QNUM || numel(x)==3), ...
                    fn_robot_input, ...
                    '   Position = ', ...
                    '   Position not valid\n' ...
                ); 
            
        end

        if numel(t) == 0
            invalid_trajectory = 1;
        elseif numel(t) == 3
            t_robot = fn_cam2robot_coords(t);
            [t_robot_q, invalid_trajectory, q] = gothere(t_robot(1),t_robot(2),t_robot(3));
            disp(q);
        elseif numel(t) == QNUM
            t_robot_q = t;
            invalid_trajectory = 0;
        end
        
    end
   
    t_robot_q_fixed = fix_target_q(t_robot_q, current_q, last_q);
    disp(t_robot_q_fixed(end,:));
    
    confirm = cmd_acquire( ...
    	'', ...
        @(x) isscalar(x) && ( x==0 || x==1 ), ...
        fn_robot_input, ...
        '   Confirm trajectory (0-1)? ', ...
        '   Ans not valid\n' ...
    ); 

end