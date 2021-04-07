function [R, t, err_lin, err_nonlin, time] = roi_pose_estimation(img, rois, i_arucos, aruco_real_sides, K, R_cam, t_cam, k, varargin)
% ROI_POSE_ESTIMATION Compute the poses of the matched ROIs in the world frame.
%
%   [R, t, err_lin, err_nonlin, time] = ROI_POSE_ESTIMATION(img, rois, i_arucos, 
%   aruco_real_sides, K, R_cam, t_cam, k)
%
%   Input arguments:
%   ------------------
%   img:                input image
%   rois:               ROIs matched with the markers
%   i_arucos:           indices of the matched markers for every ROIs
%   aruco_real_sides:   real world lengths of the sides of the markers [cm]
%   K:                  intrisics matrix of the camera (Matlab convention)
%   R_cam:              rotation matrix of the camera extrinsics in the 
%                       world frame (Matlab convention)
%   t_cam:              translation vector of the camera extrinsics in the 
%                       world frame (Matlab convention)
%   k:                  radial distortion coefficients of the camera
%   
%   Parameters:
%   --------
%   'verbose':          verbose level of the function (0, 1, 2)
%                       - 0: show nothing
%                       - 1: show the poses of the ROIs
%                       - 2: show also the markers IDs
%
%   Output arguments:
%   ------------------
%   R:                  rotation matrices of the roto-translations that map 
%                       points from the ROIs frames into the world frame
%                       (Matlab convention)
%   t:                  translation vectors of the roto-translations that map 
%                       points from the ROIs frames into the world frame 
%                       (Matlab convention)
%   err_lin:            RMS values of reprojection errors (after linear PnP)
%   err_nonlin:         RMS values of reprojection errors (after non-linear PnP)
%   time:               execution time (ignoring plots)
%
%   See also ARUCO_POSE_ESTIMATION, PNP_LIN, PNP_NONLIN

    % Start timer
    tic;

    % Default values of parameters    
    default_verbose = 1;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'verbose', default_verbose, @(x) x>=0);
    parse(p, varargin{:});
    
    % Parse function parameters
    VERBOSE = p.Results.verbose;
    
    % Generate points in the ROIs frames for PnP
    n_arucos_markers = length(aruco_real_sides);
    rois_world_pnp = cell(n_arucos_markers,1);
    for i=1:n_arucos_markers
        rois_world_pnp{i} = aruco_real_sides(i) * [
            %  x    y    z
            -0.5  0.5    0
             0.5  0.5    0
             0.5 -0.5    0
            -0.5 -0.5    0
        ];
    end
    
    % Projection matrix of the camera in the world frame
    P_cam = [R_cam; t_cam] * K;

    % Perform pose estimation
    n_rois = size(rois,1);
    % rois_undist = cell(n_rois,1);
    R_pnp = cell(n_rois,1);
    t_pnp = cell(n_rois,1);
    P_pnp = cell(n_rois,1);
    R = cell(n_rois,1);
    t = cell(n_rois,1);
    err_lin = zeros(n_rois,1);
    err_nonlin = zeros(n_rois,1);
    for i=1:n_rois

        % Remove radial distortion from ROIs (already done in pnp_nonlin)
        % rois_undist{i} = rad_dist_remove(rois{i}, K, k);
        
        % Compute the poses of the camera wrt the ROIs frames (linear PnP)
        [R_pnp{i}, t_pnp{i}, err_lin(i)] = pnp_lin(rois{i}, rois_world_pnp{i_arucos(i)}, K);
        
        % Non-linear refinement of PnP
        [R_pnp{i}, t_pnp{i}, err_nonlin(i)] = pnp_nonlin(R_pnp{i}, t_pnp{i}, rois{i}, rois_world_pnp{i_arucos(i)}, K, k);
        
        % Projection matrices associated with the ROIs frames
        P_pnp{i} = [R_pnp{i}; t_pnp{i}]*K;
        
        % Poses of the ROIs in the world frame
        % G{i}  = G_pnp{i} * inv(G_cam)
        R{i} = R_pnp{i}*R_cam'; 
        t{i} = t_pnp{i}*R_cam' - t_cam*(R_cam');
        
    end
    
    % End timer
    time = toc;
    
    % Plots
    if VERBOSE > 0
        
        figure;
        imshow(img);
        hold on;
        
        colors_rois = hsv(n_rois);
        colors_axes = ['r', 'g', 'b'];
    
        for i=1:n_rois
            
            roi = rois{i};
            roi_world_pnp = rois_world_pnp{i_arucos(i)};
            aruco_real_side = aruco_real_sides(i_arucos(i));
            
            % Plot the ROI
            if VERBOSE > 1
                lines_rois(i) = line( ...
                    [rois{i,1}(:,1); rois{i,1}(1,1)], ...
                    [rois{i,1}(:,2); rois{i,1}(1,2)], ...
                    'color', colors_rois(i,:), ...
                    'linestyle', '-', 'linewidth', 1.5, ...
                    'marker', 'o', 'markersize', 5 ...
                );
                lines_rois_str{i} = sprintf( ...
                    'ROI=%d, Aruco=%d\nX=%5.1f Y=%5.1f Z=%5.1f', ...
                    i, i_arucos(i), t{i}(1), t{i}(2), t{i}(3));
            else
                lines_rois(1) = line( ...
                     [roi(:,1); roi(1,1)], ...
                     [roi(:,2); roi(1,2)], ...
                     'color','m','linestyle','-','linewidth',1.5, ...
                     'marker','o','markersize',5);
                lines_rois_str{1} = 'ROIs detected';
            end
            
            % Plot control point
            line_control_points = plot(roi(1,1),roi(1,2), 'gs', 'MarkerSize', 10, 'LineWidth', 1);
            
            % Projection of marker centroid
            centroid_world_pnp = mean(roi_world_pnp);
            centroid_reproj = hom_tf(centroid_world_pnp, P_pnp{i}, K, k);
            
            % Plot the reprojected centroid of the marker
            % plot(centroid_reproj(1),centroid_reproj(2),'ro');
            
            % Projection of marker contours
            roi_reproj = hom_tf(roi_world_pnp, P_pnp{i}, K, k);
            
            % Plot the projected marker contours
            line_reproj_rois = plot(roi_reproj(:,1),roi_reproj(:,2),'c+');

            % Projection of marker pose axes
            axes_pose = 1.0*aruco_real_side * [
                1 0 0
                0 1 0
                0 0 1
            ];
            axes_pose_reproj = hom_tf(axes_pose, P_pnp{i}, K, k);

            % Plot the projected pose axes of the marker
            for j=1:3
                lines_axes_pose_reproj(j) = line( ...
                    [centroid_reproj(1,1) axes_pose_reproj(j,1)], ...
                    [centroid_reproj(1,2) axes_pose_reproj(j,2)], ...
                    'color',colors_axes(j),'linestyle','-','linewidth',3, ...
                    'marker','none','markersize',5);
            end

        end
        
        % Plot the world frame
        centroid_world = [0 0 0];
        axes_world = 3*[
            1 0 0
            0 1 0
            0 0 1
        ];
        centroid_world_world = hom_tf(centroid_world, P_cam, K, k);
        axes_world_reproj = hom_tf(axes_world, P_cam, K, k);

        colors_axes=['c' 'm' 'y'];
        for i=1:3
            lines_axes_world_reproj(i) = line(...
                [centroid_world_world(1,1) axes_world_reproj(i,1)], ...
                [centroid_world_world(1,2) axes_world_reproj(i,2)], ...
                'color', colors_axes(i), ...
                'linestyle','-', 'linewidth', 3, ...
                'marker','none', 'markersize', 5);
        end
        
        title('Pose Estimation ROIs');
        if n_rois > 0
            legend( [lines_rois, line_reproj_rois, line_control_points, lines_axes_pose_reproj, lines_axes_world_reproj], ...
                lines_rois_str{:}, 'ROIs reprojected', 'Control points', ...
                'ROIs poses x-axis', 'ROIs poses y-axis', 'ROIs poses z-axis', ...
                'World frame X-axis', 'World frame Y-axis', 'World frame Z-axis');
        end

    end
    
end
    