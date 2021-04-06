function [t, R, i_aruco] = get_target_from_vision(cam, vision_args, fn_robot_input)
% GET_TARGET_FROM_VISION Retrieve the position of a chosen marker in the scene
% observed by a camera (world frame).
%
%   [t, R, i_aruco] = GET_TARGET_FROM_VISION(cam, vision_args, fn_robot_input)
%
%   Input arguments:
%   ------------------
%   cam: webcam object of the camera, cf. webcam(...)
%   vision_args: struct of vision parameters, cf. below
%   fn_robot_input: function to acquire input, cf. input(...) or cmdBuffer
%
%   The struct vision_args contains the positional arguments and parameters
%   of aruco_pose_estimation(...).
%
%   Output arguments:
%   ------------------
%   t: translation vector of the roto-translation that maps points from the
%      target frame into the world frame (Matlab convention)
%   R: rotation matrix of the roto-translation that maps points from the 
%      target frame into the world frame (Matlab convention)
%   i_aruco: id of the marker correspondent to the target
%
%   See also ARUCO_POSE_ESTIMATION, GET_TARGET

    fprintf('   Acquiring image\n');
    img = snapshot(cam);

    fprintf('   Executing Aruco pose estimation\n');
    [rois, i_arucos, rois_R, rois_t] = aruco_pose_estimation( ...
        img, vision_args.aruco_markers, vision_args.aruco_real_sides, ...
        vision_args.K, vision_args.R_cam, vision_args.t_cam, vision_args.k, ...
        vision_args.options ...
    );

    n_rois = length(rois);
    for i=1:n_rois
        fprintf('      ROI %d -- Aruco id %d -- X = %g  Y = %g  Z = %g [cm]\n', i, i_arucos(i), rois_t{i}(1), rois_t{i}(2), rois_t{i}(3));
    end

    % If multiple markers are detected
    if n_rois > 1
        
        search_mode = cmd_acquire( ...
            '', ...
            @(x) isscalar(x) && ( x==1 || x==2 ), ...
            fn_robot_input, ...
            '   Looking for ROI (1) or Aruco ID (2)? ', ...
            '   Ans not valid\n' ...
        ); 
        
        % Choose target by ROI IDs
        if search_mode == 1
    
            idx = cmd_acquire( ...
                sprintf('   Choose ROI target (1-%d)\n', n_rois), ...
                @(x) x>0 && x<=n_rois, ...
                fn_robot_input, ...
                '      ROI = ', ...
                '      ROI not valid\n' ...
            ); 
            
        % Choose target by Aruco IDs
        elseif search_mode == 2
            
            aruco_ids = unique(i_arucos);
            
            id = cmd_acquire( ...
                sprintf('   Choose Aruco ID target (%s)\n', num2str(aruco_ids)), ...
                @(x) isscalar(x) && ismember(x, aruco_ids), ...
                fn_robot_input, ...
                '      Aruco ID = ', ...
                '      Aruco ID not valid\n' ...
            ); 
        
            idx_matched = find(id == i_arucos);
            
            % If multiple Arucos have the same ID, ask which one to choose
            n_idx_matched = numel(idx_matched);
            if n_idx_matched > 1
                for i=1:n_idx_matched
                    fprintf('      ROI %d -- Aruco id %d -- X = %g  Y = %g  Z = %g [cm]\n', idx_matched(i), i_arucos(idx_matched(i)), rois_t{idx_matched(i)}(1), rois_t{idx_matched(i)}(2), rois_t{idx_matched(i)}(3));
                end
                idx = cmd_acquire( ...
                    sprintf('   Choose ROI target (%s)\n', num2str(idx_matched)), ...
                    @(x) isscalar(x) && ismember(x, idx_matched), ...
                    fn_robot_input, ...
                    '      ROI = ', ...
                    '      ROI not valid\n' ...
                );
            else
                idx = idx_matched;
            end
            
        end

        t = rois_t{idx};
        R = rois_R{idx};
        i_aruco = i_arucos(idx);

    % If only one marker is detected, choose it
    elseif n_rois == 1
        
        t = rois_t{1};
        R = rois_R{1};
        i_aruco = i_arucos(1);
        
    % If no one marker is detected, print warning
    else

        fprintf('   Markers not found!!\n');
        t = [];
        R = [];
        i_aruco = [];
        
    end

end