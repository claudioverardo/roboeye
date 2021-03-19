function [P, K, intrinsics] = calibration_intrinsics_camera(dir_images, n_intrinsics, n_radial_dist)

    fprintf('------ Camera Calibration (Intrinsics) ------\n');
    fprintf('%s\n', dir_images);

    % Find images in dir_images
    files = findImages(dir_images);
    n_images = numel(files);
    
    % Path to backup of calibraton points
    m4_path = fullfile(dir_images, 'm4.mat');
    
    % Check if a backup of calibraton points exists
    if isfile(m4_path)
        
        fprintf('Found m4.mat on disk\n');
        
    % Otherwise ask the user to acquire the calibration points
    else
        
        fprintf('Acquiring m4...\n');

        % Create array of calibration points
        points_calibration = cell(1, n_images);

        % Populate points_calibration with points chosen by the user
        for idx = 1:n_images
            
            % Load image
            file = files(idx);
            image_path = fullfile(file.folder, file.name);
            image = imread(image_path);
            fprintf('File %2d: %s\n', idx, image_path);
            
            % Show image
            imshow(image);
            hold on;

            % Acquire 4 calibration points from the image
            points = [];
            for k = 1:4

                zoom on;
                pause();
                zoom off;
                
                % Acquire calibration point
                [j, i] = ginput(1);
                points = [points; [j, i]];
                
                zoom out; 

                % Plot calibration point
                plot(j, i, "r*");
                
            end

            points_calibration{1, idx} = points';
            
        end
    
        % Save coords on disk
        m4 = points_calibration; % To be used by runCalibChecker()
        save(m4_path, 'm4');
    
    end
    
    % Calculate the P camera matrices and the K intrinsic matrices
    fprintf('--------- Starting runCalibChecker ----------\n');
    [P, K, intrinsics] = runCalibChecker(files, n_intrinsics, n_radial_dist);
    
    % Save results
    save(fullfile(dir_images, 'P'), 'P');
    save(fullfile(dir_images, 'K'), 'K');
    save(fullfile(dir_images, 'intrinsics'), 'intrinsics');
    
end
