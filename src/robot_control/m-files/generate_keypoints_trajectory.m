function [t_robot_q, confirm] = generate_keypoints_trajectory(method, current_q, last_q, cam, vision_args, trajectory_planning_args, objects_dict, fn_cam2robot_coords, fn_robot_input)
% GENERATE_KEYPOINTS_TRAJECTORY TODO
%
%   [t_robot_q_fixed, t_robot_q, confirm] = GENERATE_KEYPOINTS_TRAJECTORY(current_q, last_q, cam, vision_args, fn_cam2robot_coords, fn_robot_input)
%
%   Input arguments:
%   ------------------
%   current_q:
%   last_q:
%   cam:
%   vision_args: 
%   fn_cam2robot_coords:
%   fn_robot_input:
%
%   Output arguments:
%   ------------------
%   t_robot_q_fixed:   
%   t_robot_q:
%   confirm: 
%
%   See also TODO

    QNUM = numel(current_q);
    
    invalid_trajectory = 1;
    while invalid_trajectory
        
        fprintf('\n----- Generate keypoints trajectory -----\n');
        fprintf('Method: %s\n', method);
        
        if strcmp(method, 'move-q')
        
            t_robot_q = cmd_acquire( ...
                '', ...
                @(x) isvector(x) && ( ( numel(x)==QNUM && check_limits_joints(x) ) || numel(x)==3 ), ...
                fn_robot_input, ...
                '   Insert position [M1,M2,M3,M4,M5,M6]: ', ...
                '   Position not valid\n' ...
            ); 
            t_robot_q = fix_target_q(t_robot_q, current_q, last_q);
            invalid_trajectory = 0;
        
        elseif strcmp(method, 'move-n')
            
            n_points = cmd_acquire( ...
                '', ...
                @(x) isscalar(x) && isnumeric(x) && x>0, ...
                fn_robot_input, ...
                'Number of points: ', ...
                'Number not valid\n' ...
            );
        
            t_points = zeros(n_points,3);        
            for i=1:n_points
                t_points(i,:) = cmd_acquire( ...
                    '', ...
                    @(x) isvector(x) && numel(x)==3, ...
                    fn_robot_input, ...
                    sprintf('   Point %d [X Y Z]: ', i), ...
                    '   Point not valid\n' ...
                );
            end
            
            fprintf('Computing trajectory...\n');
            t_robot_points = zeros(n_points,3);
            t_robot_q = zeros(n_points,QNUM);
            invalid_trajectories = zeros(1,n_points);
            
            for i=1:n_points
                t_robot_points(i,:) = fn_cam2robot_coords(t_points(i,:));
                [t_robot_q(i,:), invalid_trajectories(i), ~] = gothere( ...
                    trajectory_planning_args.braccio_params, ...
                    t_robot_points(i,1),t_robot_points(i,2),t_robot_points(i,3),90,73,0,[], ...
                    'verbose', trajectory_planning_args.gothere_verbose ...
                 );
                fprintf('   Point %d [M1,M2,M3,M4,M5,M6]: %s\n', i, mat2str(t_robot_q(i,:), 3));
            end
            invalid_trajectory = any(invalid_trajectories);
            
        elseif strcmp(method,'move') == 1 || strcmp(method,'grasp') == 1 || strcmp(method,'grasp-parabola') == 1

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
                        '   Insert target position [X,Y,Z]\n', ...
                        @(x) isvector(x) && numel(x)==3, ...
                        fn_robot_input, ...
                        '   Position = ', ...
                        '   Position not valid\n' ...
                    ); 
                    i_aruco = 0;

            end

            switch numel(t)

                case 0
                    invalid_trajectory = 1; % when vision does not find anything

                case 3
                    t_robot = fn_cam2robot_coords(t);
                    fprintf('   Target (camera frame): X = %g  Y = %g  Z = %g [cm]\n', t(1), t(2), t(3));
                    fprintf('   Target (robot frame):  X = %g  Y = %g  Z = %g [mm]\n', t_robot(1), t_robot(2), t_robot(3));

                    if strcmp(method,'move') == 1

                        fprintf('   Computing trajectory...\n');
                        [t_robot_q, invalid_trajectory, t_robot_q_enc] = gothere( ...
                            trajectory_planning_args.braccio_params, ...
                            t_robot(1),t_robot(2),t_robot(3),90,73,0,[], ...
                            'verbose', trajectory_planning_args.gothere_verbose ...
                         );
                        t_robot_q = fix_target_q(t_robot_q, current_q, last_q);

                        % fprintf('   Target (encoder positions):  %s\n', mat2str(t_robot_q_enc, 3));
                        fprintf('   Target (joints): %s\n', mat2str(t_robot_q(end,:), 3));

                    elseif strcmp(method,'grasp') == 1 || strcmp(method,'grasp-parabola') == 1

                        if i_aruco == 0 % default offsets
                            offset_h = 0;
                            offset_r = 0;
                            offset_ef = 0;
                            fprintf('   Object unknown');
                        else
                            offset_h = objects_dict(i_aruco).offset_h;
                            offset_r = objects_dict(i_aruco).offset_r;
                            offset_ef = objects_dict(i_aruco).offset_ef;
                            fprintf('   Object detected: %s\n', objects_dict(i_aruco).name);
                        end

                        fprintf('      Applying offsets [dh,dr,def] = [%g,%g,%g]\n',offset_h,offset_r,offset_ef);
                        nz = [0 0 1];
                        delta_t_robot = offset_objects(offset_h, offset_r, t_robot, nz);

                        % Target position fine-tuning
                        delta_t_robot_fine = cmd_acquire( ...
                            '', ...
                            @(x) isscalar(x) || (isvector(x) && numel(x)==0), ...
                            fn_robot_input, ...
                            '      Fine-tuning offset? [dx,dy,dz] = ', ...
                            '      Offset not valid\n' ...
                        ); 

                        % Applying the offsets
                        t_robot_off = t_robot + delta_t_robot + delta_t_robot_fine;
                        if t_robot_off(3) < 0
                            fprintf('      WARNING: Z=%g < 0, forcing to 0\n', t_robot_off(3));
                            t_robot_off(3) = 0;
                        end
                        fprintf('   Target corrected (robot frame):  X = %g  Y = %g  Z = %g [mm]\n', t_robot_off(1), t_robot_off(2), t_robot_off(3));

                        fprintf('   Computing pick trajectory...\n');
                        [t_robot_q, invalid_trajectory, t_robot_q_enc] = gothere( ...
                            trajectory_planning_args.braccio_params, ...
                            t_robot_off(1),t_robot_off(2),t_robot_off(3),90,0,offset_ef,[], ...
                            'verbose', trajectory_planning_args.gothere_verbose ...
                        );

                        if ~invalid_trajectory

                            t_robot_q = fix_target_q(t_robot_q, current_q, last_q);

                            % fprintf('   Target (encoder positions):  %s\n', mat2str(t_robot_q_enc, 3));

                            fprintf('        pre pick  (joints): %s\n', mat2str(t_robot_q(end,:), 3));
                            t_robot_q = [t_robot_q; t_robot_q(end,1:QNUM-1) 73];
                            fprintf('       post pick  (joints): %s\n', mat2str(t_robot_q(end,:), 3));

                            if strcmp(method,'grasp') == 1

                                t_dest_q = trajectory_planning_args.box_coords_grasp;

                                fprintf('   Computing place trajectory...\n');
                                t_robot_q = [t_robot_q; [t_robot_q(end,1) t_dest_q(2:QNUM)]];
                                t_robot_q = [t_robot_q; fix_target_q(t_dest_q, t_robot_q(end,:))];
                                fprintf('        pre place (joints): %s\n', mat2str(t_robot_q(end,:), 3));
                                t_robot_q = [t_robot_q; t_dest_q(1:QNUM-1) 0];
                                fprintf('       post place (joints): %s\n', mat2str(t_robot_q(end,:), 3));

                            elseif strcmp(method,'grasp-parabola') == 1

                                t_dest = trajectory_planning_args.box_coords_grasp_parabola;
                                t_dest_robot = fn_cam2robot_coords(t_dest);

                                fprintf('   Computing place trajectory (parabola)...\n');
                                [q_parab, invalid_trajectory] = parabolic_traj( ...
                                    t_robot_off, t_dest_robot, 'auto', t_robot_q(end,QNUM-1), 10, ...
                                    trajectory_planning_args.braccio_params, ... 
                                    t_robot_q(end,QNUM), offset_ef,...
                                    trajectory_planning_args.parabolic_traj_verbose ...
                                 );
                                t_robot_q = [t_robot_q; q_parab];
                                for i = 1:size(q_parab,1)
                                    fprintf('       parab_traj %d (joints): %s\n', i, mat2str(q_parab(i,:), 3));
                                end
                                t_robot_q = [t_robot_q; t_robot_q(end,1:QNUM-1) 0];
                                fprintf('       post place (joints): %s\n', mat2str(t_robot_q(end,:), 3));

                            end

                        end

                    end

            end
        
        end

        if invalid_trajectory
            fprintf('   ERROR: trajectory not valid!\n');
        end
        
    end
    
    confirm = cmd_acquire( ...
    	'', ...
        @(x) isscalar(x) && ( x==0 || x==1 ), ...
        fn_robot_input, ...
        '   Confirm trajectory (0-1)? ', ...
        '   Ans not valid\n' ...
    ); 

end