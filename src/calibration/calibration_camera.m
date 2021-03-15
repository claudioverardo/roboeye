function [P, K, intrinsics] = calibration_camera(num_images, dir_images)

    fprintf('-------- Camera Calibration --------\n');
    fprintf('%s\n', dir_images);
    
    % Path to backup of calibraton points
    m4_path = fullfile(dir_images, 'm4.mat');
    
    % Check if
    if isfile(m4_path)
        
        fprintf('Found m4 on disk\n');
        
    else
        
        fprintf('Acquiring m4...\n');

        % Create array of calibration points
        points_calibration = cell(1, num_images);

        % Populate points_calibration with points chosen by the user
        for idx = 1:num_images

            image_filename = sprintf('%02d.png', idx);
            image_path = fullfile(dir_images, image_filename);
            
            fprintf('File %2d: %s\n', idx, image_path);
            
            image = imread(image_path);
            imshow(image);
            hold on;

            % Acquire 4 calibration points from a single image
            points = [];
            for k = 1:4

                zoom on;
                pause();
                zoom off;
                [local_j, local_i] = ginput(1);
                points = [points; [local_j, local_i]];
                zoom out; 

                % Plot calibration point
                plot(local_j, local_i, "r*");
            end

            points_calibration{1, idx} = points';
            
        end
    
        % Save coords on disk
        m4 = points_calibration; % Fusiello library
        save(m4_path, 'm4');
    
    end
    
    % Calculate the P camera matrices and the K intrinsic matrices
    fprintf('----- Starting runCalibChecker -----\n');
    [P, K, intrinsics] = runCalibChecker(num_images, dir_images);
    
    % Save results
    save(fullfile(dir_images, 'P'), 'P');
    save(fullfile(dir_images, 'K'), 'K');
    save(fullfile(dir_images, 'intrinsics'), 'intrinsics');
    
end
