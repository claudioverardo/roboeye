function [R_cam, t_cam] = calibration_extrinsics_camera(cam, K, k, step_size, grid_arrangement, cm2px_scale, dir, check)
% CALIBRATION_EXTRINSICS_CAMERA Retrive the rotation matrix and the translation
% vector (extrinsics) of a camera wrt a world frame attached to a checkerboard.
%
%   [R_cam, t_cam] = CALIBRATION_EXTRINSICS_CAMERA(cam, K, k, step_size, grid_arrangement, cm2px_scale, dir, check)
%
%   Input arguments:
%   ------------------
%   cam:                webcam object (cf. webcam(...))
%   K:                  intrinsics matrix of the camera (literature convention)
%   k:                  radial distortion coefficients of the camera
%   step_size:          side of the squares of the checkerboard [cm]
%   grid_arrangement    [x-steps y-steps] steps of the checkerboard along x,y axes
%   cm2px_scale:        dimension in cm of 1 pixel of the rectified image
%   dir:                directory where to write/read the calibration files
%   check:              boolean, if true checks the calibration on a new image
%
%   Output arguments:
%   ------------------
%   R_cam:              rotation matrix of the camera extrinsics in the world frame
%                       (literature convention)
%   t_cam:              translation vector of the camera extrinsics in the world frame
%                       (literature convention)
%   
%   NOTE: this function requires the following packages:
%         - MATLAB Support Package for USB Webcams
%         - Computer Vision Toolkit (http://www.diegm.uniud.it/fusiello/demo/toolkit/)
%
%   See also CALIBRATION_INTRINSICS_CAMERA, PNP_LIN, PNP_NONLIN

    if nargin <= 7
       check = false; 
    end

    fprintf('\n------ Camera Calibration (Extrinsics) ------\n');
    fprintf('%s\n', dir);

    if ~exist(dir, 'dir')
       mkdir(dir);
    end
    
    new_setup = 0;
    
    img_path = fullfile(dir, 'checker.png');
    
    % Check if a the image of the checkerboard already exists
    if isfile(img_path)
        fprintf('Found checker.png on disk\n');
        img = imread(img_path);
    
    % Otherwise acquire it from the camera
    else
        fprintf('Acquiring checker.png ...\n');
        img = snapshot(cam);
        imwrite(img, img_path);
        
        new_setup = 1;
    end
    
    files = findImages(dir);
    
    fig = figure;
    imshow(img);
    hold on;
    
    control_points_path = fullfile(dir, 'control_points.mat');
    
    % Check if a backup of grid control points exists
    if isfile(control_points_path) && ~new_setup
        
        fprintf('Found control_points.mat on disk\n');
        load(control_points_path, 'control_points');
        
        for i = 1:4
            line_control_points = plot(control_points(1,i), control_points(2,i), 'r*');
        end
        
    % Otherwise ask the user to acquire the grid control points
    else
        
        fprintf('Acquiring control_points...\n');
        points = [];
        for i = 1:4
            title('Press any key to start acquisition of a new control point');
            zoom on;
            pause();
            title('Click to acquire a new control point');
            zoom off;
            [x, y] = ginput(1);
            points = [points; [x, y]];
            zoom out; 
            
            line_control_points = plot(x, y, 'r*');
            fprintf('point %d acquired\n', i);
        end
    
        % Store acquired points as grid control points for the image
        control_points = points';
        save(control_points_path, 'control_points');
        
        new_setup = 1;

    end
    
    R_cam_path = fullfile(dir, 'R_cam.mat');
    t_cam_path = fullfile(dir, 't_cam.mat');
    
    % Check if the camera extrinsics are already calibrated
    if isfile(R_cam_path) && isfile(t_cam_path) && ~new_setup
        
        fprintf('Found R_cam.mat, t_cam.mat on disk\n');
        load(R_cam_path, 'R_cam');
        load(t_cam_path, 't_cam');
        
    % Otherwise start calibration
    else 
            
        fprintf('Computing R_cam, t_cam...\n');

        % Generate world coordinates for the grid points
        M_grid = generateGridPoints(grid_arrangement, step_size, 'Checker');

        % Detect grid points in the image
        m_grid = findGridPoints(rgb2gray(img), 'Checker', M_grid(1:2,:), control_points, grid_arrangement, files(1), cm2px_scale);

        % Remove radial distortion (already done in pnp_nonlin)
        % [m_grid_undist, err_dist] = rad_dist_remove(m_grid', K', k);
        % fprintf('Distortion error (RMS): %g\n', err_dist);
        
        % Estimate the pose of the camera wrt the checkerboard (linear PnP)
        [R_cam, t_cam, reproj_err_lin] = pnp_lin(m_grid', M_grid', K');
        fprintf('Reproj error (RMS) ___lin: %f\n', reproj_err_lin);
        
        % Non-linear refinement of PnP
        [R_cam, t_cam, reproj_err_nonlin] = pnp_nonlin(R_cam, t_cam, m_grid', M_grid', K', k);
        fprintf('Reproj error (RMS) nonlin: %f\n', reproj_err_nonlin);
        
        % Save the pose on disk (literature convention)
        R_cam = R_cam';
        t_cam = t_cam';
        save(R_cam_path, 'R_cam');
        save(t_cam_path, 't_cam');
        
    end
    
    % Plot the reprojection of the world frame axes onto the image
    P = K*[R_cam,t_cam];
    centroid_world = [0 0 0];
    axes_world = step_size * [1 0 0; 0 1 0; 0 0 1];
    centroid_image = hom_tf(centroid_world, P', K', k);
    axes_image = hom_tf(axes_world, P', K', k);
    colors_axes=['c' 'm' 'y'];
    
    figure(fig);
    for i=1:3
        lines_axes(i) = line( ...
            [centroid_image(1,1) axes_image(i,1)], ...
            [centroid_image(1,2) axes_image(i,2)], ...
            'color', colors_axes(i), ...
            'linestyle','-', 'linewidth', 3, ...
            'marker','none', 'markersize', 5);
    end
    legend([line_control_points lines_axes], ...
        'Control points', 'World frame X-axis', 'World frame Y-axis', 'World frame Z-axis');
    title('Calibrated camera extrinsics');
    
    if check
        
        img_check = snapshot(cam);
        
        figure;
        imshow(img_check);
        hold on;
    
        line_control_points_check = plot(control_points(1,:), control_points(2,:), 'r*');
        for i=1:3
            lines_axes_check(i) = line( ...
                [centroid_image(1,1) axes_image(i,1)], ...
                [centroid_image(1,2) axes_image(i,2)], ...
                'color', colors_axes(i), ...
                'linestyle','-', 'linewidth', 3, ...
                'marker','none', 'markersize', 5);
        end
        legend([line_control_points_check lines_axes_check], ...
            'Control points', 'World frame X-axis', 'World frame Y-axis', 'World frame Z-axis');
        title('Check camera extrinsics');
        
    end
    
end