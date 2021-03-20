function [R, t] = calibration_extrinsics_camera(cam, K, dir, cm2px_scale)

    fprintf('------ Camera Calibration (Extrinsics) ------\n');
    fprintf('%s\n', dir);

    if ~exist(dir, 'dir')
       mkdir(dir);
    end
    
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
    end
    
    files = findImages(dir);
    
    figure;
    imshow(img);
    hold on;
    
    control_points_path = fullfile(dir, 'control_points.mat');
    
    % Check if a backup of grid control points exists
    if isfile(control_points_path)
        
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
            zoom on;
            pause();
            zoom off;
            [j, i] = ginput(1);
            points = [points; [j, i]];
            zoom out; 
            line_control_points = plot(j, i, 'r*');
        end
    
        % Store acquired points as grid control points for the image
        control_points = points';
        save(control_points_path, 'control_points');

    end
    
    R_path = fullfile(dir, 'R.mat');
    t_path = fullfile(dir, 't.mat');
    
    if isfile(R_path) && isfile(t_path)
        
        fprintf('Found R.mat, t.mat on disk\n');
        load(R_path, 'R');
        load(t_path, 't');
        
    else 
        
        fprintf('Computing R, t...\n');
        
        % Generate world coordinates for the grid points
        step_size = 3; % side of the squarein centimeters
        grid_arrangement = [8, 6];  % rows by columns
        M_grid = generateGridPoints(grid_arrangement, step_size, 'Checker');

        % Detect grid points in the image
        m_grid = findGridPoints(rgb2gray(img), 'Checker', M_grid(1:2,:), control_points, files(1), cm2px_scale);

        % Estimate the pose of the camera wrt the checkerboard
        % rand_indices = randsample(size(M_grid,2),20);
        [R, t, reproj_err_lin] = pnp_lin(m_grid', M_grid', K');
        [R, t, reproj_err_nonlin] = pnp_nonlin(R, t, m_grid', M_grid', K');
        fprintf('Reproj error (RMS) ___lin: %f\n', reproj_err_lin);
        fprintf('Reproj error (RMS) nonlin: %f\n', reproj_err_nonlin);
    
        % Save the pose on disk
        save(R_path, 'R');
        save(t_path, 't');
        
    end
    
    % Plot the reprojection of the frame axes onto the image
    X_world = 3*[1 0 0; 0 1 0; 0 0 1; 0 0 0];
    X_image = homography(X_world, [R;t]*K');
    
    figure(1);
    colors_axes=['r' 'g' 'b'];
    for i=1:3
        lines_axes(i) = line([X_image(end,1) X_image(i,1)], ...
            [X_image(end,2) X_image(i,2)], ...
            'color', colors_axes(i), ...
            'linestyle','-', 'linewidth', 3, ...
            'marker','none', 'markersize', 5);
    end
    
    legend([line_control_points lines_axes], 'control points', 'pose x-axis', 'pose y-axis', 'pose z-axis');

end