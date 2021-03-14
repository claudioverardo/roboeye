function [rois, i_arucos, R, t] = aruco_pose_estimation(img, K, aruco_markers, aruco_real_side, aruco_detection_parameters, varargin)
    
    % NB: points and K with Matlab convention

    % Default values of parameters
    default_verbose = 1;

    % Input parser
    p = inputParser;
    addParameter(p, 'verbose', default_verbose);
    parse(p, varargin{:});
    
    % Parse function parameters
    VERBOSE = p.Results.verbose;
    
    % Launch Aruco Detection
    fprintf('-------- Aruco Detection --------\n');
    [rois, i_arucos, ~] = aruco_detection(...
        img, aruco_markers, ...
        aruco_detection_parameters{:} ...
    );

    % Launch Aruco PnP
    fprintf('----------- Aruco PnP -----------\n');
    
    % Real world coordinates of points
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
        [R{i}, t{i}] = roi_pnp(rois{i}, roi_world, K);
    end
    
    % Plots
    if VERBOSE > 0
        
        figure;
        imshow(img);
        hold on;
    
        for i=1:n_rois
            roi = rois{i};

            % Projective matrix
            P = [R{i}; t{i}]*K;
        
            % Project the centroid of the marker
            centroid_world = mean(roi_world);
            % centroid_proj = htx(P', centroid_world')'; % [Fusiello]
            centroid_proj = homography(centroid_world, P);
            
            % Plot the projected centroid of the marker
            line([roi(:,1); roi(1,1)], ...
                 [roi(:,2); roi(1,2)], ...
                 'color','r','linestyle','-','linewidth',1.5, ...
                 'marker','o','markersize',5);
            plot(roi(1,1),roi(1,2), 'go', 'MarkerFaceColor', 'g');
            plot(centroid_proj(1),centroid_proj(2),'go');
            
            % Check projection of ROI points
            % roi_proj = htx(P', roi_world')'; % [Fusiello]
            roi_proj = homography(roi_world, P);
            plot(roi_proj(:,1),roi_proj(:,2),'go');

            % Project the pose axes of the marker
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
                line([centroid_proj(1,1) axes_proj(j,1)], ...
                     [centroid_proj(1,2) axes_proj(j,2)], ...
                     'color',colors(j),'linestyle','-','linewidth',1.5, ...
                     'marker','o','markersize',5);
            end

        end

    end

end