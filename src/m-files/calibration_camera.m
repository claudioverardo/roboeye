function [P, K] = calibration_camera(image_folder, image_number)
    % Define constats
    calibration_image_path = strcat(image_folder, '/%02d.png');

    % Create coords calibration array
    coords_calibration = cell(1, image_number);

    % Populate array coords with input points from developer
    for idx = 1:image_number
        image = imread(sprintf(calibration_image_path, idx));
        imshow(image);

        points = [];

        for k = 1:4
            zoom on;
            pause();
            zoom off;
            [local_j, local_i] = ginput(1);
            points = [points; [local_j, local_i]];
            zoom out; 

            hold on;

            % Plot calibration point
            plot(local_j, local_i, "r*");
        end

        coords_calibration{1, idx} = points';
    end
    
    % Save coords on disk
    m4 = coords_calibration;
    save(strcat(image_folder, '/m4'), 'm4');
    
    % Calculate the P and K camera matrices
    [P, K] = run_calib_checker(image_folder, image_number);
end