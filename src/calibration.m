% Calibration for camera 1
clear;
num_images = 4;
coords_calibration_1 = cell(1, num_images);

for idx = 1:num_images
    image1 = imread(sprintf("./calibrationImages/01/images1_%02d.png", idx));
    imshow(image1);
    
    points = [];
    
    for k = 1:4
        zoom on;
        pause() 
        zoom off;
        [local_j, local_i] = ginput(1);
        points = [points; [local_j, local_i]];
        zoom out; 
        
        hold on;
        
        % Plot calibration point
        plot(local_j, local_i, "r*");
    end
    
    coords_calibration_1{1, idx} = points';
end


m4 = coords_calibration_1;
save("./calibrationImages/01/m4", "m4");

[P1, K1] = run_calib_checker("./calibrationImages/01", num_images);
save("./calibrationImages/01/calibration1", "P1", "K1");