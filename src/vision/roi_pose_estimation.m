function [R, t] = roi_pose_estimation(img, rois, i_arucos, aruco_real_sides, K, R_cam, t_cam, varargin)
% ROI_POSE_ESTIMATION Compute pose of matched ROIs in the camera frame
%
%   [R, t] = ROI_POSE_ESTIMATION(img, rois, i_arucos, aruco_real_sides, K, R_cam, t_cam)
%
%   Input arguments:
%   ------------------
%   img:                input image
%   rois:               regions of interest matched with the markers
%   i_arucos:           indices of the matched marker for every rois matched 
%   aruco_real_sides:   lengths of the markers in the dictionary [cm]
%   K:                  intrisics matrix of the camera (Matlab convention)
%   R_cam:              rotation matrix of the camera pose in the world frame
%                       (Matlab convention)
%   t_cam:              translation matrix of the camera pose in the world frame
%                       (Matlab convention)
%   
%   Parameters:
%   --------
%   'verbose':          verbose level of the function (allowed values 0, 1, 2)
%
%   Output arguments:
%   ------------------
%   R:                  rotation matrices of the rois poses in the world frame
%                       (Matlab convention)
%   t:                  translation vectors of the rois poses in the world frame
%                       (Matlab convention)
%
%   See also ARUCO_POSE_ESTIMATION

    % Default values of parameters    
    default_verbose = 1;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'verbose', default_verbose, @(x) x>=0);
    parse(p, varargin{:});
    
    % Parse function parameters
    VERBOSE = p.Results.verbose;
    
    % Generate world points for PnP
    n_arucos_markers = length(aruco_real_sides);
    rois_world_pnp = cell(n_arucos_markers,1);
    for i=1:n_arucos_markers
        % Real world coordinates of ROIs
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
    R_pnp = cell(n_rois,1);
    t_pnp = cell(n_rois,1);
    P_pnp = cell(n_rois,1);
    R = cell(n_rois,1);
    t = cell(n_rois,1);
    for i=1:n_rois
        
        % Poses of the camera wrt the ROI frames (PnP + non-linear refinement)
        [R_pnp{i}, t_pnp{i}, reproj_err_lin] = pnp_lin(rois{i}, rois_world_pnp{i_arucos(i)}, K);
        [R_pnp{i}, t_pnp{i}, reproj_err_nonlin] = pnp_nonlin(R_pnp{i}, t_pnp{i}, rois{i}, rois_world_pnp{i_arucos(i)}, K);
        % Projective matrices associated with the ROI frames
        P_pnp{i} = [R_pnp{i}; t_pnp{i}]*K;
        
        if VERBOSE > 1
            fprintf('ROI %d: reproj error (RMS) lin: %f -- nonlin: %f\n', i, reproj_err_lin, reproj_err_nonlin);
        end
        
        % Poses of the ROIs in the world frame
        % G{i}  = G_pnp{i} * inv(G_cam)
        R{i} = R_pnp{i}*R_cam'; 
        t{i} = t_pnp{i}*R_cam' - t_cam*(R_cam');
        
    end
    
    % Plots
    if VERBOSE > 0
        
        figure;
        imshow(img);
        hold on;
        
        colors_rois = autumn(n_arucos_markers);
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
                    'color', colors_rois(i_arucos(i),:), ...
                    'linestyle', '-', 'linewidth', 1.5, ...
                    'marker', 'o', 'markersize', 5 ...
                );
                lines_rois_str{i} = sprintf( ...
                    'ROI=%d, Aruco=%d\nX=%5.1f Y=%5.1f Z=%5.1f', ...
                    i, i_arucos(i), t{i}(1), t{i}(2), t{i}(3));
            else
                lines_rois(1) = line([roi(:,1); roi(1,1)], ...
                     [roi(:,2); roi(1,2)], ...
                     'color','m','linestyle','-','linewidth',1.5, ...
                     'marker','o','markersize',5);
                lines_rois_str{1} = 'Detected ROIs';
            end
            
            % Plot control point
            line_control_points = plot(roi(1,1),roi(1,2), 'gs', 'MarkerSize', 10, 'LineWidth', 1); % 'MarkerFaceColor', 'g'
            
            % Projection of marker centroid
            centroid_world_pnp = mean(roi_world_pnp);
            % centroid_proj = htx(P_pnp{i}', centroid_world_pnp')'; % [Fusiello]
            centroid_reproj = homography(centroid_world_pnp, P_pnp{i});
            
            % Plot the reprojected centroid of the marker
            % plot(centroid_reproj(1),centroid_reproj(2),'ro');
            
            % Projection of marker contours
            % roi_reproj = htx(P_pnp{i}', roi_world_pnp')'; % [Fusiello]
            roi_reproj = homography(roi_world_pnp, P_pnp{i});
            
            % Plot the projected marker contours
            line_reproj_rois = plot(roi_reproj(:,1),roi_reproj(:,2),'c+');

            % Projection of marker pose axes
            axes_world_pnp = 1.0*aruco_real_side * [
                1 0 0
                0 1 0
                0 0 1
            ];
            % axes_proj = htx(P_pnp{i}', axes_world_pnp')'; % [Fusiello]
            axes_reproj = homography(axes_world_pnp, P_pnp{i});

            % Plot the projected pose axes of the marker
            for j=1:3
                lines_reproj_axes(j) = line([centroid_reproj(1,1) axes_reproj(j,1)], ...
                     [centroid_reproj(1,2) axes_reproj(j,2)], ...
                     'color',colors_axes(j),'linestyle','-','linewidth',3, ...
                     'marker','none','markersize',5);
            end

        end
        
        % Plot the world frame        
        X_world = 3*[1 0 0; 0 1 0; 0 0 1; 0 0 0];
        X_image = homography(X_world, P_cam);

        colors_axes=['r' 'g' 'b'];
        for i=1:3
            lines_reproj_axes(i) = line([X_image(end,1) X_image(i,1)], ...
                [X_image(end,2) X_image(i,2)], ...
                'color', colors_axes(i), ...
                'linestyle','-', 'linewidth', 3, ...
                'marker','none', 'markersize', 5);
        end
        
        title('Pose of ROIs');
        if n_rois > 0
            legend( [lines_rois, line_reproj_rois, line_control_points, lines_reproj_axes], ...
                lines_rois_str{:}, 'Reprojected ROIs', 'Control points', 'Pose x-axis', 'Pose y-axis', 'Pose z-axis');
        end
        
        % TEST: find coordinates of ROI_2 wrt frame of ROI_1
        % T1 = [R{1} zeros(3,1); t{1} 1];
        % T2 = [R{2} zeros(3,1); t{2} 1];
        % delta_T = T2*inv(T1);
        % x_world = homography(0.5*[0 0 0; -3 3 0; 3 3 0; 3 -3 0; -3 -3 0], delta_T);
        % x_image = homography(x_world, P{1});
        % plot(x_image(:,1),x_image(:,2), 'yo', 'MarkerFaceColor', 'y');

    end
    
end
    