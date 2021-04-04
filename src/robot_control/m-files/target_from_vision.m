function [t, R, i_aruco] = target_from_vision(cam, vision_args, fn_robot_input)
% TARGET_FROM_VISION TODO
%
%   [t, R, i_aruco] = TARGET_FROM_VISION(cam, vision_args, fn_robot_input)
%
%   Input arguments:
%   ------------------
%   cam:
%   vision_args:
%   fn_robot_input:
%
%   Output arguments:
%   ------------------
%   t:   
%   R: 
%   i_aruco: 
%
%   See also TODO

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
        fprintf('      ROI %d -- Aruco id %d -- X = %g  Y = %g  Z = %g [cm]\n', i, i_arucos(i), rois_t{i}(1), rois_t{i}(2), rois_t{i}(3));
    end

    if n_rois > 1
        
        search_mode = cmd_acquire( ...
            '', ...
            @(x) isscalar(x) && ( x==1 || x==2 ), ...
            fn_robot_input, ...
            '   Looking for ROI (1) or Aruco id (2)? ', ...
            '   Ans not valid\n' ...
        ); 
    
        if search_mode == 1
    
            idx = cmd_acquire( ...
                sprintf('   Choose ROI target (1-%d)\n', n_rois), ...
                @(x) x>0 && x<=n_rois, ...
                fn_robot_input, ...
                '      ROI = ', ...
                '      ROI not valid\n' ...
            ); 
            
        elseif search_mode == 2
            
            aruco_ids = unique(i_arucos);
            
            id = cmd_acquire( ...
                sprintf('   Choose Aruco id target (%s)\n', num2str(aruco_ids)), ...
                @(x) isscalar(x) && ismember(x, aruco_ids), ...
                fn_robot_input, ...
                '      Aruco id = ', ...
                '      Aruco id not valid\n' ...
            ); 
        
            idx_matched = find(id == i_arucos);
            
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

    elseif n_rois == 1
        
        t = rois_t{1};
        R = rois_R{1};
        i_aruco = i_arucos(1);
        
    else

        fprintf('   Markers not found!!\n');
        t = [];
        R = [];
        i_aruco = [];
        
    end

end