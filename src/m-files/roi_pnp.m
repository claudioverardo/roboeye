function [R, t] = roi_pnp(img, rois, aruco_real_side, K_obj, VERBOSE)

    % NB: points and K with Matlab convention
    
    % Real world coordinates of ROIs
    n_rois = size(rois,1);
    roi_world = aruco_real_side * [
        %  x    y    z
        -0.5  0.5    0
         0.5  0.5    0
         0.5 -0.5    0
        -0.5 -0.5    0
    ];

    % Perform PnP
    R = cell(n_rois,1);
    t = cell(n_rois,1);
    for i=1:n_rois
        [R{i}, t{i}] = pnp(rois{i}, roi_world, K_obj);
    end
    K = K_obj.IntrinsicMatrix;
    
    % Plots
    if VERBOSE > 0
        
        figure;
        imshow(img);
        hold on;
    
        for i=1:n_rois
            roi = rois{i};
            
            % Plot the ROI + control point
            line_roi = line([roi(:,1); roi(1,1)], ...
                 [roi(:,2); roi(1,2)], ...
                 'color','m','linestyle','-','linewidth',1.5, ...
                 'marker','o','markersize',5);
            line_control_point = plot(roi(1,1),roi(1,2), 'go', 'MarkerFaceColor', 'g');

            % Projective matrix
            P = [R{i}; t{i}]*K;
        
            % Projection of marker centroid
            centroid_world = mean(roi_world);
            % centroid_proj = htx(P', centroid_world')'; % [Fusiello]
            centroid_proj = homography(centroid_world, P);
            
            % Plot the projected centroid of the marker
            % plot(centroid_proj(1),centroid_proj(2),'ro');
            
            % Projection of marker contours
            % roi_proj = htx(P', roi_world')'; % [Fusiello]
            roi_proj = homography(roi_world, P);
            
            % Plot the projected marker contours
            line_projected_roi = plot(roi_proj(:,1),roi_proj(:,2),'yo');

            % Projection of marker pose axes
            axes_world = 1.5*aruco_real_side * [
                1 0 0
                0 1 0
                0 0 1
            ];
            % axes_proj = htx(P', axes_world')'; % [Fusiello]
            axes_proj = homography(axes_world, P);

            % Plot the projected pose axes of the marker
            colors = ['r', 'g', 'b'];
            for j=1:3
                line_axis(j) = line([centroid_proj(1,1) axes_proj(j,1)], ...
                     [centroid_proj(1,2) axes_proj(j,2)], ...
                     'color',colors(j),'linestyle','-','linewidth',1.5, ...
                     'marker','o','markersize',5);
            end

        end
        
        legend( [line_roi, line_projected_roi, line_control_point, line_axis], ...
            'Detected ROIs', 'Projected ROIs', 'Control points', 'Pose x-axis', 'Pose y-axis', 'Pose z-axis');
        title('PnP ROIs');

    end
    
end
    