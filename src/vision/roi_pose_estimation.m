function [R, t] = roi_pose_estimation(img, rois, i_arucos, aruco_real_sides, K, varargin)
% ROI_POSE_ESTIMATION Compute pose of matched ROIs in the camera frame
%
%   [R, t] = ROI_POSE_ESTIMATION(img, rois, i_arucos, aruco_real_sides, K)
%
%   Input arguments:
%   ------------------
%   img:                input image
%   rois:               region of interest matched with the markers
%   i_arucos:           indices of the matched marker for every rois matched 
%   aruco_real_sides:   lengths of the markers in the dictionary [cm]
%   K:                  intrisics matrix of the camera (Matlab convention)
%   
%   Parameters:
%   --------
%   'verbose:           verbose level of the function (allowed values 0, 1, 2, 3)
%
%   Output arguments:
%   ------------------
%   R:                  rotation matrix of the roi pose (Matlab convention)
%   t:                  translation vector of the roi pose (Matlab convention)
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
    rois_world = cell(n_arucos_markers,1);
    for i=1:n_arucos_markers
        % Real world coordinates of ROIs
        rois_world{i} = aruco_real_sides(i) * [
            %  x    y    z
            -0.5  0.5    0
             0.5  0.5    0
             0.5 -0.5    0
            -0.5 -0.5    0
        ];
    end

    % Perform PnP
    n_rois = size(rois,1);
    R = cell(n_rois,1);
    t = cell(n_rois,1);
    P = cell(n_rois,1);
    for i=1:n_rois
        
        % Pose (rotation + translation)
        [R{i}, t{i}, reproj_err_lin] = pnp_lin(rois{i}, rois_world{i_arucos(i)}, K);
        % Non linear refinement of the pose
        [R{i}, t{i}, reproj_err_nonlin] = pnp_nonlin(R{i}, t{i}, rois{i}, rois_world{i_arucos(i)}, K);
        % Projective matrix
        P{i} = [R{i}; t{i}]*K;
        
        if VERBOSE > 1
            fprintf('ROI %d: reproj error (RMS) lin: %f -- nonlin: %f\n', i, reproj_err_lin, reproj_err_nonlin);
        end
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
            roi_world = rois_world{i_arucos(i)};
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
                lines_rois_str{i} = sprintf('ROI=%d, Aruco=%d', i, i_arucos(i));
            else
                lines_rois(1) = line([roi(:,1); roi(1,1)], ...
                     [roi(:,2); roi(1,2)], ...
                     'color','m','linestyle','-','linewidth',1.5, ...
                     'marker','o','markersize',5);
                lines_rois_str{1} = 'Detected ROIs';
            end
            
            % Plot control point
            line_control_point = plot(roi(1,1),roi(1,2), 'gs', 'MarkerSize', 10, 'LineWidth', 1); % 'MarkerFaceColor', 'g'
            
            % Projection of marker centroid
            centroid_world = mean(roi_world);
            % centroid_proj = htx(P', centroid_world')'; % [Fusiello]
            centroid_proj = homography(centroid_world, P{i});
            
            % Plot the projected centroid of the marker
            % plot(centroid_proj(1),centroid_proj(2),'ro');
            
            % Projection of marker contours
            % roi_proj = htx(P', roi_world')'; % [Fusiello]
            roi_proj = homography(roi_world, P{i});
            
            % Plot the projected marker contours
            line_projected_roi = plot(roi_proj(:,1),roi_proj(:,2),'c+');

            % Projection of marker pose axes
            axes_world = 1.0*aruco_real_side * [
                1 0 0
                0 1 0
                0 0 1
            ];
            % axes_proj = htx(P', axes_world')'; % [Fusiello]
            axes_proj = homography(axes_world, P{i});

            % Plot the projected pose axes of the marker
            for j=1:3
                lines_axes(j) = line([centroid_proj(1,1) axes_proj(j,1)], ...
                     [centroid_proj(1,2) axes_proj(j,2)], ...
                     'color',colors_axes(j),'linestyle','-','linewidth',3, ...
                     'marker','none','markersize',5);
            end

        end
        
        title('Pose of ROIs');
        if n_rois > 0
            legend( [lines_rois, line_projected_roi, line_control_point, lines_axes], ...
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
    