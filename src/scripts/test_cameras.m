% webcamlist

%% Test camera 1
cam1 = webcam(1);
% cam1.AvailableResolutions
% cam1.Resolution = '1280x720';
% cam1.Brightness = 50;
preview(cam1);

%% Test camera 2
cam2 = webcam(2);
% cam2.AvailableResolutions
% cam2.Resolution = '1280x720';
% cam2.Brightness = 50;
preview(cam2);

%% Test stereo acquisition 
image1 = snapshot(cam1);
image2 = snapshot(cam2);

subplot(2,1,1); imshow(image1); title('Camera 1');
subplot(2,1,2); imshow(image2); title('Camera 2');
