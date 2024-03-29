function [trajectory, time_trajectory, confirm] = generate_trajectory(method, home_q, current_q, delta_t, cam, vision_args, trajectory_planning_args, fn_cam2robot_coords, fn_robot_input)
% GENERATE_TRAJECTORY High level interface to generate robot trajectories.
% Trajectories defined pointwise (P) and via keypoints (K) can be generated. 
% The latter ones require a low level controller that interpolate between
% keypoints to be executed on the robot.
%
%   [trajectory, time_trajectory, confirm] = GENERATE_KEYPOINTS_TRAJECTORY(
%   method, current_q, delta_t, cam, vision_args, trajectory_planning_args,
%   fn_cam2robot_coords, fn_robot_input)
%   
%   Input arguments:
%   ------------------
%   method: method used to generate the trajectory
%       - 'back-home': go back to the home position (K)
%       - 'move-q': move to a position in joints space (K)
%       - 'move-t-npoints': move to n positions in 3D space (K)
%       - 'move-t-pointwise': move to a position in 3D space from home (P)
%       - 'move-t': move to a position in 3D space (K)
%       - 'grasp': grasp a object in a position in 3D space (K)
%       - 'grasp-parabola': as 'grasp', with a parabolic trajectory (K)
%   home_q: 1xQNUM array, home position of the robot (joints)
%   current_q: 1xQNUM array, current position of the robot (joints)
%   delta_t: timestep of the trajectory execution [ms]
%   cam: webcam object of the camera, cf. webcam(...)
%   vision_args: struct of vision parameters, cf. get_target_from_vision(...)
%   trajectory_planning_args: struct of trajectory planning parameters
%   fn_cam2robot_coords: function to convert points from vision to robot frame 
%   fn_robot_input: function to acquire input, cf. input(...) or cmdBuffer
%
%   trajectory_planning_args struct
%   -------------------------------
%   - braccio_params: real parameters of the robot, cf. direct_kin(...)
%   - z_min: minimum z-value of target points [mm], in robot frame
%   - joint_safety_radius: minimum distance that joints have to mantain from 
%     the ground [mm], in robot frame
%   - box_coords_grasp: destination of 'grasp' [cm], in vision frame
%   - box_coords_grasp_parabola: as above but for 'grasp-parabola' [cm]
%   - touchdown_verbose: verbosity level of touchdown(...)
%   - gothere_verbose: verbosity level of gothere(...)
%   - parabolic_traj_verbose: verbosity level of parabolic_traj(...)
%   - verbose: verbosity level of generate_trajectory(...)
%   - objects_dict: parameters of the objects to be grasped, cf. object_offset(...)
%
%   Output arguments:
%   ------------------
%   trajectory: NxQNUM array of the generated N-points trajectory
%   time_trajectory: estimated execution time of the trajectory
%   confirm: flag to confirm or cancel execution of the trajectory
%
%   NOTE: this function requires the MATLAB Support Package for USB Webcams.
%   For details regarding vision_args refers to get_target_from_vision(...).
%
%   See also ROBOT_FSM_INTERFACE, EMULATE_KEYPOINTS_TRAJECTORY

    QNUM = numel(current_q);
    Z_MIN = trajectory_planning_args.z_min;
    
    invalid_trajectory = 1;
    while invalid_trajectory
        
        fprintf('\n--------- Generate trajectory ---------\n');
        fprintf('Method: %s\n', method);
        
        switch method
            
            % Go back to home position
            % NOTE: keypoints trajectory (K)
            case 'back-home'
                
                type_trajectory = 'keypoints';
                t_robot_q = fix_target_q(home_q, current_q);
                invalid_trajectory = 0;
            
            % Acquire a position in joint space and reach it
            % NOTE: keypoints trajectory (K)
            case 'move-q'
                
                type_trajectory = 'keypoints';
                t_robot_q = get_target('q', QNUM, [], [], fn_robot_input);
                t_robot_q = fix_target_q(t_robot_q, current_q);
                invalid_trajectory = 0;
            
            % Acquire n positions in 3d space (vision frame) and reach them
            % NOTE: keypoints trajectory (K)
            case 'move-t-npoints'
                
                type_trajectory = 'keypoints';
                t_points = get_target('3d-npoints', [], [], [], fn_robot_input);
                n_points = size(t_points,1);
                
                t_robot_points = zeros(n_points,3);
                for i=1:n_points
                    t_robot_points(i,:) = fn_cam2robot_coords(t_points(i,:));
                    if t_robot_points(i,3) < Z_MIN
                        fprintf('   WARNING: point %d with Z=%g < %g mm (robot frame), forcing to %g\n', i, t_robot_points(i,3), Z_MIN, Z_MIN);
                        t_robot_points(i,3) = Z_MIN;
                    end
                end

                fprintf('Computing trajectory...\n');
                t_robot_q = zeros(n_points,QNUM);
                invalid_trajectories = zeros(1,n_points);
                for i=1:n_points
                    [t_robot_q(i,:), invalid_trajectories(i), ~] = gothere( ...
                        trajectory_planning_args.braccio_params, ...
                        t_robot_points(i,1), t_robot_points(i,2), t_robot_points(i,3), ...
                        90, 73, 0, [], trajectory_planning_args.post_corr, home_q, ...
                        'verbose', trajectory_planning_args.gothere_verbose ...
                     );
                    fprintf('   Point %d [M1,M2,M3,M4,M5,M6]: %s\n', i, mat2str(t_robot_q(i,:), 3));
                end
                
                invalid_trajectory = any(invalid_trajectories);
            
            % Acquire a position in 3d space (vision frame) and reach it.
            % WARNING: the robot must start in home position!!!
            % NOTE: pointwise trajectory (P)
            case 'move-t-pointwise'
                
                type_trajectory = 'pointwise';
                t = get_target('3d-vision', [], cam, vision_args, fn_robot_input);

                t_robot = fn_cam2robot_coords(t);
                fprintf('   Target (camera frame): X = %g  Y = %g  Z = %g [cm]\n', t(1), t(2), t(3));
                fprintf('   Target (robot frame):  X = %g  Y = %g  Z = %g [mm]\n', t_robot(1), t_robot(2), t_robot(3));
                
                if t_robot(3) < Z_MIN
                    fprintf('   WARNING: Z=%g < %g mm (robot frame), forcing to %g\n', t_robot(3), Z_MIN, Z_MIN);
                    t_robot(3) = Z_MIN;
                end
                
                fprintf('   Computing trajectory...\n');
                [t_robot_q, invalid_trajectory] = touchdown( ...
                    trajectory_planning_args.braccio_params, ...
                    t_robot(1), t_robot(2), t_robot(3), ...
                    trajectory_planning_args.post_corr, home_q, ...
                    trajectory_planning_args.touchdown_verbose ...
                 );
                fprintf('   Target (joints): %s\n', mat2str(t_robot_q(end,:), 3));
            
            % Acquire a position in 3d space (vision frame) and reach it.
            % NOTE: keypoints trajectory (K)
            case 'move-t'
                
                type_trajectory = 'keypoints';
                t = get_target('3d-vision', [], cam, vision_args, fn_robot_input);

                t_robot = fn_cam2robot_coords(t);
                fprintf('   Target (camera frame): X = %g  Y = %g  Z = %g [cm]\n', t(1), t(2), t(3));
                fprintf('   Target (robot frame):  X = %g  Y = %g  Z = %g [mm]\n', t_robot(1), t_robot(2), t_robot(3));

                if t_robot(3) < Z_MIN
                    fprintf('   WARNING: Z=%g < %g mm (robot frame), forcing to %g\n', t_robot(3), Z_MIN, Z_MIN);
                    t_robot(3) = Z_MIN;
                end
                
                fprintf('   Computing trajectory...\n');
                [t_robot_q, invalid_trajectory, t_robot_q_enc] = gothere( ...
                    trajectory_planning_args.braccio_params, ...
                    t_robot(1), t_robot(2), t_robot(3), ...
                    90, 73, 0, [], trajectory_planning_args.post_corr, home_q, ...
                    'verbose', trajectory_planning_args.gothere_verbose ...
                 );
                t_robot_q = fix_target_q(t_robot_q, current_q);
                fprintf('   Target (joints): %s\n', mat2str(t_robot_q(end,:), 3));

                fprintf('   Target (encoder positions):  %s\n', mat2str(t_robot_q_enc, 3));
            
            % Acquire the position of an object in 3d space (vision frame), grasp it and bring it to a destination box.
            % The object->box trajectory is a parabola if 'grasp-parabola' is chosen.
            % NOTE: keypoints trajectory (K)
            case {'grasp','grasp-parabola'}
                
                type_trajectory = 'keypoints';
                [t, i_aruco] = get_target('3d-vision', [], cam, vision_args, fn_robot_input);

                t_robot = fn_cam2robot_coords(t);
                fprintf('   Target (camera frame): X = %g  Y = %g  Z = %g [cm]\n', t(1), t(2), t(3));
                fprintf('   Target (robot frame):  X = %g  Y = %g  Z = %g [mm]\n', t_robot(1), t_robot(2), t_robot(3));

                if i_aruco == 0 % default offsets
                    offset_h = 0;
                    offset_r = 0;
                    offset_ef = 0;
                    fprintf('   Object unknown\n');
                else
                    offset_h = trajectory_planning_args.objects_dict(i_aruco).offset_h;
                    offset_r = trajectory_planning_args.objects_dict(i_aruco).offset_r;
                    offset_ef = trajectory_planning_args.objects_dict(i_aruco).offset_ef;
                    fprintf('   Object detected: %s\n', trajectory_planning_args.objects_dict(i_aruco).name);
                end

                fprintf('      Applying offsets [dh,dr,def] = [%g,%g,%g]\n',offset_h,offset_r,offset_ef);
                delta_t_robot = object_offset(offset_h, offset_r, t_robot);

                % Target position fine-tuning
                delta_t_robot_fine = cmd_acquire( ...
                    '', ...
                    @(x) (isscalar(x) && x==0) || (isvector(x) && numel(x)==3), ...
                    fn_robot_input, ...
                    '      Fine-tuning offset? [dx,dy,dz] = ', ...
                    '      Offset not valid\n' ...
                ); 

                % Applying the offsets
                t_robot_off = t_robot + delta_t_robot + delta_t_robot_fine;
                if t_robot_off(3) < Z_MIN
                    fprintf('      WARNING: Z=%g < %g mm (robot frame), forcing to %g\n', t_robot_off(3), Z_MIN, Z_MIN);
                    t_robot_off(3) = Z_MIN;
                end
                fprintf('   Target corrected (robot frame):  X = %g  Y = %g  Z = %g [mm]\n', t_robot_off(1), t_robot_off(2), t_robot_off(3));

                fprintf('   Computing pick trajectory...\n');
                [t_robot_q, invalid_trajectory, t_robot_q_enc] = gothere( ...
                    trajectory_planning_args.braccio_params, ...
                    t_robot_off(1), t_robot_off(2), t_robot_off(3), ...
                    90, 0, offset_ef, [], trajectory_planning_args.post_corr, home_q, ...
                    'verbose', trajectory_planning_args.gothere_verbose ...
                );
                t_robot_q = fix_target_q(t_robot_q, current_q);
                
                fprintf('        pre pick  (joints): %s\n', mat2str(t_robot_q(end,:), 3));
                t_robot_q = [t_robot_q; t_robot_q(end,1:QNUM-1) 73];
                fprintf('       post pick  (joints): %s\n', mat2str(t_robot_q(end,:), 3));
                
                if ~invalid_trajectory && strcmp(method,'grasp') == 1

                    t_dest = trajectory_planning_args.box_coords_grasp;
                    t_dest_robot = fn_cam2robot_coords(t_dest);

                    fprintf('   Computing place trajectory...\n');
                    [t_dest_q, invalid_trajectory, t_dest_q_enc] = gothere( ...
                        trajectory_planning_args.braccio_params, ...
                        t_dest_robot(1), t_dest_robot(2), t_dest_robot(3), ...
                        90, 73, 0, [], trajectory_planning_args.post_corr, home_q, ...
                        'verbose', trajectory_planning_args.gothere_verbose ...
                    );
                    % t_dest_q = trajectory_planning_args.box_coords_grasp;

                    t_robot_q = [t_robot_q; [t_robot_q(end,1) t_dest_q(2:QNUM)]];
                    t_robot_q = [t_robot_q; fix_target_q(t_dest_q, t_robot_q(end,:))];
                    fprintf('        pre place (joints): %s\n', mat2str(t_robot_q(end,:), 3));
                    t_robot_q = [t_robot_q; t_dest_q(1:QNUM-1) 0];
                    fprintf('       post place (joints): %s\n', mat2str(t_robot_q(end,:), 3));

                elseif ~invalid_trajectory && strcmp(method,'grasp-parabola') == 1

                    t_dest = trajectory_planning_args.box_coords_grasp_parabola;
                    t_dest_robot = fn_cam2robot_coords(t_dest);

                    fprintf('   Computing place trajectory (parabola)...\n');
                    [q_parab, invalid_trajectory] = parabolic_traj( ...
                        t_robot_off, t_dest_robot, 'auto', t_robot_q(end,QNUM-1), ...
                        trajectory_planning_args.n_points_parabola, ...
                        trajectory_planning_args.braccio_params, ... 
                        t_robot_q(end,QNUM), offset_ef, ... 
                        trajectory_planning_args.post_corr, home_q, ...
                        trajectory_planning_args.parabolic_traj_verbose ...
                     );
                 
                    t_robot_q = [t_robot_q; q_parab];
                    for i = 1:size(q_parab,1)
                        fprintf('       parabolic_traj %d (joints): %s\n', i, mat2str(q_parab(i,:), 3));
                    end
                    t_robot_q = [t_robot_q; t_robot_q(end,1:QNUM-1) 0];
                    fprintf('       post place (joints): %s\n', mat2str(t_robot_q(end,:), 3));

                end

        end

        if invalid_trajectory
            fprintf('   ERROR: trajectory not valid!\n');
        end
        
    end
    
    trajectory = t_robot_q;
    
    % Emulate the trajectory performed by the robot
    if strcmp(type_trajectory,'pointwise')
        trajectory_robot = trajectory;
        key_idcs = [];
    elseif strcmp(type_trajectory,'keypoints')
        fprintf('   Emulating robot trajectory\n');
        [trajectory_robot, key_idcs] = emulate_keypoints_trajectory(current_q, trajectory);
        fprintf('   Num trajectory keypoints: %d\n', size(trajectory,1)); 
    end
    fprintf('   Num trajectory points: %d\n', size(trajectory_robot,1)); 
    
    % Convert trajectory from robot to model convention
    trajectory_robot_mod_coords = braccio_angles_inv( ...
        trajectory_robot(:,1:QNUM-1), ...
        trajectory_planning_args.post_corr, ...
        home_q(1:QNUM-1) ...
    );
    
    % Plot the trajectory
    if trajectory_planning_args.verbose > 0
        fprintf('   Plotting trajectory\n');
        plot_config( ...
            trajectory_robot_mod_coords, ...
            trajectory_planning_args.braccio_params, ...
            key_idcs ...
        );
        title('Generated trajectory');
        % plot_config_rob( ... % just for checking
        %     trajectory_robot(:,1:QNUM-1), ...
        %     trajectory_planning_args.braccio_params, ...
        %     trajectory_planning_args.post_corr, ...
        %     home_q, ...
        %     key_idcs ...
        % );
    end
    
    % Check if there are singularities in the trajectory performed by the robot
    fprintf('   Checking singularities\n');
    [sing_flag, sing_vec] = check_sing( ...
        trajectory_robot_mod_coords, ...
        trajectory_planning_args.braccio_params ...
    );
    sing_idcs = find(sing_vec);
    
    % home_position is singular? yes, but actually no...
    % remove it from singular positions if detected
    i = 1;
    while i <= length(sing_idcs)
        if all(trajectory_robot(sing_idcs(i),:) == home_q)
            sing_vec(sing_idcs(i)) = 0;
            sing_idcs(i) = [];
        else
            i = i+1;
        end
    end
    sing_flag = any(sing_vec);
    
    % Show the singular positions
    if sing_flag
        trajectory_robot_sing = trajectory_robot(sing_idcs,:);
        for i=1:length(sing_idcs)
            fprintf('        SINGULARITY WARNING: position %d is singular: %s\n', ...
                    sing_idcs(i), mat2str(trajectory_robot_sing(i,:)) ...
            );
        end
    end
    
    % Collision checking
    fprintf('   Checking collisions\n');
    j = 1;
    collision_idcs = [];
    for i = 1:size(trajectory_robot,1)
        Joint_position = joints_pos( ...
            trajectory_robot_mod_coords(i,:), ...
            trajectory_planning_args.braccio_params ...
        );
        if any(Joint_position(:,3) < trajectory_planning_args.z_min+trajectory_planning_args.joint_safety_radius)
            collision_idcs(j) = i;
            j=j+1;
        end
    end
    coll_flag = any(collision_idcs);
    
    % Show collision positions
    if coll_flag
        trajectory_robot_coll = trajectory_robot(collision_idcs,:);
        for i=1:length(collision_idcs)
            fprintf('       COLLISION WARNING: position %d cause a collision: %s\n', ...
                    collision_idcs(i), mat2str(trajectory_robot_coll(i,:)) ...
            );
        end
    end
        
    % Estimate the execution time of the trajectory
    % time_trajectory = estimate_time_trajectory(type_trajectory, trajectory, current_q, delta_t);
    time_trajectory = estimate_time_trajectory('pointwise', trajectory_robot, current_q, delta_t);
    
    % Confirm or cancel the execution of the trajectory
    if nargout > 2
        if ~strcmp(method,'back-home') || sing_flag || coll_flag
            confirm = cmd_acquire( ...
                '', ...
                @(x) isscalar(x) && ( x==0 || x==1 ), ...
                fn_robot_input, ...
                '   Confirm trajectory (0-1)? ', ...
                '   Ans not valid\n' ...
            );
        else
            confirm = 1;
        end
    end

end