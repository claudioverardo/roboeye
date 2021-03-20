function images = acquire_calibration_images(n_images, cameras, dirs_images)
% ACQUIRE_CALIBRATION_IMAGES Acquire some images from a set of cameras
%   to perform camera's calibrations with the SMZ algorithm
%
%   images = ACQUIRE_CALIBRATION_IMAGES(n_images, cameras, dirs_images)
%
%   Input arguments:
%   ------------------
%   n_images:       number of the images to be acquire
%   cameras:        array of camera objects (cf webcam)
%   dirs_images:    cell array of the folder names where the script 
%                   save the images
%
%   Output arguments:
%   ------------------
%   images:         acquired images 
%                   images{i,j} the i-th image acquired from the j-th camera
%   
%   NOTE to use this script you need the MATLAB Support Package for USB Webcams
%
%   See also WEBCAMLIST, WEBCAM, SNAPSHOT, PREVIEW

    num_cameras = length(cameras);
    images = cell(n_images,num_cameras);

    fprintf('---- Acquire calibration images [%d cameras] ----\n', num_cameras);
    
    for j = 1:num_cameras
        if ~exist(dirs_images{j}, 'dir')
           mkdir(dirs_images{j})
        end
    end
    
    fprintf('START in');
    print_countdown(8);
    
    for i = 1:n_images
        
        fprintf('Acquisition %d ...', i);
        
        for j = 1:num_cameras
            
            images{i,j} = snapshot(cameras{j});
            filename_image = sprintf('%02d.png', i);
            imwrite(images{i,j}, fullfile(dirs_images{j}, filename_image));
            
        end

        if i < n_images
            fprintf(' next in');
            print_countdown(3);
        else
            fprintf(' done\n');
        
    end

end
