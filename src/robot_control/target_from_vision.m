function [t, R, i_aruco] = target_from_vision(cam, vision_args, fn_robot_input)

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

    if n_rois > 0
        
        idx = cmd_acquire( ...
            sprintf('   Choose ROI target (1-%d)\n', n_rois), ...
            @(x) x>0 && x<=n_rois, ...
            fn_robot_input, ...
            '      ROI = ', ...
            '      ROI not valid\n' ...
        ); 

        t = rois_t{idx};
        R = rois_R{idx};
        i_aruco = i_arucos(idx);
        
    else

        fprintf('   Markers not found!!\n');
        t = [];
        R = [];
        i_aruco = [];
        
    end

end