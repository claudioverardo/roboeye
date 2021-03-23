function [R_cam, t_cam] = calibration_extrinsics_camera(cam, K, step_size, grid_arrangement, cm2px_scale, dir)
% CALIBRATION_EXTRINSICS_CAMERA Retrive the rotation matrix and the translation
% vector (extrinsics) of a camera wrt a world frame attached to a checkerboard.
%
%   [R_cam, t_cam] = CALIBRATION_EXTRINSICS_CAMERA(cam, K, step_size, grid_arrangement, cm2px_scale, dir)
%
%   Input arguments:
%   ------------------
%   cam:                webcam object (cf. webcam(...))
%   K:                  intrinsics matrix of the camera (literature convention)
%   step_size:          side of the squares of the checkerboard [cm]
%   grid_arrangement    [x-steps y-steps] steps of the checkerboard along x,y axes
%   cm2px_scale:        dimension in cm of 1 pixel of the rectified image
%   dir:                directory where to write/read the calibration files
%
%   Output arguments:
%   ------------------
%   R_cam:              rotation matrix of the camera extrinsics in the world frame
%                       (literature convention)
%   t_cam:              translation vector of the camera extrinsics in the world frame
%                       (literature convention)
%   
%   NOTE this function requires the following packages:
%        - MATLAB Support Package for USB Webcams
%        - Computer Vision Toolkit (http://www.diegm.uniud.it/fusiello/demo/toolkit/)
%
%   See also CALIBRATION_INTRINSICS_CAMERA, PNP_LIN, PNP_NONLIN

    fprintf('------ Camera Calibration (Extrinsics) ------\n');
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
        
        for k = 1:4
            line_control_points = plot(control_points(1,k), control_points(2,k), 'r*');
        end
        
    % Otherwise ask the user to acquire the grid control points
    else
        
        fprintf('Acquiring control_points...\n');
        points = [];
        for k = 1:4
            title('Press any key to start acquisition of a new control point');
            zoom on;
            pause();
            title('Click to acquire a new control point');
            zoom off;
            [j, i] = ginput(1);
            points = [points; [j, i]];
            zoom out; 
            
            line_control_points = plot(j, i, 'r*');
            fprintf('point %d done\n', k);
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

        % Estimate the pose of the camera wrt the checkerboard
        % rand_indices = randsample(size(M_grid,2),20);
        [R_cam, t_cam, reproj_err_lin] = pnp_lin(m_grid', M_grid', K');
        [R_cam, t_cam, reproj_err_nonlin] = pnp_nonlin(R_cam, t_cam, m_grid', M_grid', K');
        fprintf('Reproj error (RMS) ___lin: %f\n', reproj_err_lin);
        fprintf('Reproj error (RMS) nonlin: %f\n', reproj_err_nonlin);
        R_cam = R_cam';
        t_cam = t_cam';
    
        % Save the pose on disk
        save(R_cam_path, 'R_cam');
        save(t_cam_path, 't_cam');
        
    end
    
    % Plot the reprojection of the world frame axes onto the image
    X_world = 2 * step_size * [1 0 0; 0 1 0; 0 0 1; 0 0 0]';
    X_image = htx(K*[R_cam,t_cam], X_world);
    
    figure(fig);
    colors_axes=['r' 'g' 'b'];
    for i=1:3
        lines_axes(i) = line([X_image(1,end) X_image(1,i)], ...
            [X_image(2,end) X_image(2,i)], ...
            'color', colors_axes(i), ...
            'linestyle','-', 'linewidth', 3, ...
            'marker','none', 'markersize', 5);
    end
    
    legend([line_control_points lines_axes], ...
        'Control points', 'World frame x-axis', 'World frame y-axis', 'World frame z-axis');
    title('Calibrated camera extrinsics');
    
end