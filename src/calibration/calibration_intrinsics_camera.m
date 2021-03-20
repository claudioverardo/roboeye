function [P, K, intrinsics] = calibration_intrinsics_camera(dir_images, n_intrinsics, n_radial_dist, cm2px_scale)
% CALIBRATION_INTRINSICS_STEREO Retrive the intrisics of a camera via 
%   SMZ calibration (wrapper of the runCalibChecker)
%
%   [P, K, intrinsics] = CALIBRATION_INTRINSICS_STEREO(dir_images, n_intrinsics, n_radial_dist, cm2px_scale)
%
%   Input arguments:
%   ------------------
%   dir_images:     folder containing the images fot the SMZ calibration    
%   n_intrinsics:   number of intrisics to be calibrated (4, 5)
%                   4: fx, fy, u0, v0
%                   5: fx, fy, u0, v0, skew
%   n_radial_dist:  number of the distortion coefficient to be calibrated (1, 2)
%   cm2px_scale:    dimension in cm of 1 pixel of the rectified image           
%
%   Output arguments:
%   ------------------
%   P:              projection matrices of the cameras (literature convention)
%   K:              calibrated intrisics matrices (literature convention)
%   intrinsics:     table with intrinsics parameters and radial distortion
%                   coefficients 
%
%   NOTE to use this script you need Computer Vision Toolkit 
%        (http://www.diegm.uniud.it/fusiello/demo/toolkit/)
%
%   See also RUNCALIBCHECKER

    fprintf('------ Camera Calibration (Intrinsics) ------\n');
    fprintf('%s\n', dir_images);

    % Find images in dir_images
    files = findImages(dir_images);
    n_images = numel(files);
    
    % Path to backup of grid control points
    control_points_path = fullfile(dir_images, 'control_points.mat');
    
    % Check if a backup of grid control points exists
    if isfile(control_points_path)
        
        fprintf('Found control_points.mat on disk\n');
        load(control_points_path, 'control_points');
        
    % Otherwise ask the user to acquire the grid control points
    else
        
        fprintf('Acquiring control_points...\n');

        % Create array of grid control points
        control_points = cell(1, n_images);

        % Populate control_points with points chosen by the user
        for idx = 1:n_images
            
            % Load image
            file = files(idx);
            image_path = fullfile(file.folder, file.name);
            image = imread(image_path);
            fprintf('File %2d: %s\n', idx, image_path);
            
            % Show image
            imshow(image);
            hold on;

            % Acquire 4 points from the image
            points = [];
            for k = 1:4
                
                % Acquire point
                zoom on;
                pause();
                zoom off;
                [j, i] = ginput(1);
                points = [points; [j, i]];
                zoom out; 

                % Plot point
                plot(j, i, "r*");
                
            end
            
            % Store acquired points as grid control points for the image
            control_points{1, idx} = points';
            
        end
    
        % Save control_points on disk
        save(control_points_path, 'control_points');
    
    end
    
    % Calculate the P camera matrices and the K intrinsic matrices
    fprintf('--------- Starting runCalibChecker ----------\n');
    [P, K, intrinsics] = runCalibChecker(files, control_points, n_intrinsics, n_radial_dist, cm2px_scale);
    
    % Save results
    save(fullfile(dir_images, 'P'), 'P');
    save(fullfile(dir_images, 'K'), 'K');
    save(fullfile(dir_images, 'intrinsics'), 'intrinsics');
    
end
