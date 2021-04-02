function [trajectory, confirm] = generate_custom_trajectory(cam, vision_args, fn_cam2robot_coords, fn_robot_input)
    
    invalid_trajectory = 1;
    while invalid_trajectory

        fprintf('\n------ Generate custom trajectory ------\n');

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
                    '   Insert target position\n', ...
                    @(x) isvector(x) && numel(x)==3, ...
                    fn_robot_input, ...
                    '   [X,Y,Z] = ', ...
                    '   Position not valid\n' ...
                ); 

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
    
    confirm = cmd_acquire( ...
    	'Confirm trajectory (0-1)?\n', ...
        @(x) isscalar(x) && ( x==0 || x==1 ), ...
        fn_robot_input, ...
        'Ans: ', ...
        'Ans not valid\n' ...
    ); 

end