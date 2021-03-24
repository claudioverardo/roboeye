function test = check_epipolar_geometry(cam1, cam2, F)
% CHECK_EPIPOLAR_GEOMETRY Acquire two points from the two images of a stereo
% pair and compute the Longuet-Higgins equation between them.
%
%   test = CHECK_EPIPOLAR_GEOMETRY(cam1, cam2, F)
%
%   Input arguments:
%   ------------------
%   cam1:   camera object of the first camera (cf. webcam(...))
%   cam2:   camera object of the second camera (cf. webcam(...))
%   F:      fundamental matrix of the stereo pair (cam1 assumed as reference)
%
%   Output arguments:
%   ------------------
%   test:   value of the Longuet-Higgins equation p2'*F*p1, where p1, p2 are 
%           the points acquired from the first and second camera respectively
%           (in homogeneous coordinates)
%
%   See also CALIBRATION_EXTRINSICS_STEREO, CALIBRATION_EXTRINSICS_STEREO_SMZ

    % Acquire 1st image
    img1 = snapshot(cam1);
    
    % Acquire 2nd image
    img2 = snapshot(cam2);
    
    % Get point in the 1st image
    figure;
    imshow(img1);
    hold on;
    title('Press any key to start acquisition of a new point');
    zoom on;
    pause();
    title('Click to acquire a new point');
    zoom off;
    [j1, i1] = ginput(1);
    zoom out; 
    plot(j1, i1, 'r*');
    title('Aquired point - 1st image');
    fprintf('p1: [%f %f]\n', j1, i1);
    
    % Get point in the 2nd image
    figure;
    imshow(img2);
    hold on;
    title('Press any key to start acquisition of a new point');
    zoom on;
    pause();
    title('Click to acquire a new point');
    zoom off;
    [j2, i2] = ginput(1);
    zoom out;
    plot(j2, i2, 'r*');
    title('Aquired point - 2nd image');
    fprintf('p2: [%f %f]\n', j2, i2);

    % Build homogeneous image points
    p1 = [j1 i1 1]';
    p2 = [j2, i2, 1]';

    % Longuet-Higgins equation
    test = p2' * F * p1;
    fprintf('test p2''*F*p1: %f\n', test);

end