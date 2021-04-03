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
                    @(x) isvector(x) && ( ( numel(x)==QNUM && check_limits_joints(x) ) || numel(x)==3 ), ...
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
                        t_robot(1),t_robot(2),t_robot(3),90,73,0, ...
                        'verbose', trajectory_planning_args.verbose ...
                     );
                    
                elseif strcmp(method,'grasp') == 1
                    
                    if i_aruco == 0 % default offsets
                        offset_h = 0;
                        offset_r = 0;
                        offset_ef = 0;
                    else
                        offset_h = objects_dict(i_aruco).offset_h;
                        offset_r = objects_dict(i_aruco).offset_r;
                        offset_ef = objects_dict(i_aruco).offset_ef;
                    end
                    
                    nz = [0 0 1];
                    delta_t_robot = offset_objects(offset_h, offset_r, t_robot, nz);
                    
                    delta_t_robot_fine = cmd_acquire( ...
                        '', ...
                        @(x) isscalar(x) || (isvector(x) && numel(x)==0), ...
                        fn_robot_input, ...
                        '   Insert z offset: ', ...
                        '   Offset not valid\n' ...
                    ); 
                
                    % t_off = t + [0 0 offset_z+delta_offset_z];
                    % t_robot_off = fn_cam2robot_coords(t_off);
                    % fprintf('   Target (camera frame): X = %g  Y = %g  Z = %g [cm]\n', t_off(1), t_off(2), t_off(3));
                    
                    t_robot_off = t_robot + delta_t_robot + delta_t_robot_fine;
                    fprintf('   Target (robot frame):  X = %g  Y = %g  Z = %g [mm]\n', t_robot_off(1), t_robot_off(2), t_robot_off(3));
                
                    fprintf('   Computing trajectory...\n');
                    [t_robot_q, invalid_trajectory, t_robot_q_enc] = gothere( ...
                        trajectory_planning_args.braccio_params, ...
                        t_robot_off(1),t_robot_off(2),t_robot_off(3),90,0,offset_ef, ...
                        'verbose', trajectory_planning_args.verbose ...
                    );
                
                end
                
                fprintf('   Target (encoder positions):  %s\n', mat2str(t_robot_q_enc, 3));
                
            case QNUM
                t_robot_q = t;
                invalid_trajectory = 0;
        
        end
        
    end
   
    t_robot_q = fix_target_q(t_robot_q, current_q, last_q);
    
    if strcmp(method,'move') == 1
        fprintf('   Target (joint positions):  %s\n', mat2str(t_robot_q(end,:), 3));
    elseif strcmp(method,'grasp') == 1
        fprintf('   Target  pre pick  1(joint positions):  %s\n', mat2str(t_robot_q(end,:), 3));
        t_robot_q = [t_robot_q; t_robot_q(end,1:QNUM-1) 73];
        fprintf('   Target post pick  (joint positions):  %s\n', mat2str(t_robot_q(end,:), 3));
        
        dest_q = [23 117 0 122 90 73];
        t_robot_q = [t_robot_q; [t_robot_q(end,1) dest_q(2:QNUM)]];
        t_robot_q = [t_robot_q; fix_target_q(dest_q, t_robot_q(end,:))];
        fprintf('   Target  pre place (joint positions):  %s\n', mat2str(t_robot_q(end,:), 3));
        t_robot_q = [t_robot_q; dest_q(1:QNUM-1) 0];
        fprintf('   Target post place (joint positions):  %s\n', mat2str(t_robot_q(end,:), 3));
    end
    
    confirm = cmd_acquire( ...
    	'', ...
        @(x) isscalar(x) && ( x==0 || x==1 ), ...
        fn_robot_input, ...
        '   Confirm trajectory (0-1)? ', ...
        '   Ans not valid\n' ...
    ); 

end