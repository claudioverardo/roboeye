function [delta_R, delta_t, E, F] = calibration_extrinsics_stereo(K1, R1, t1, K2, R2, t2, dir)
% CALIBRATION_EXTRINSICS_STEREO Retrieve the extrinsics and epipolar matrices
% of a stereo pair. The two cameras are assumed with known intrinsics and
% extrinsics wrt the same world frame.
%
%   [delta_R, delta_t, E, F] = CALIBRATION_EXTRINSICS_STEREO(K1, R1, t1, K2, R2, t2, dir)
%
%   Input arguments:
%   ------------------
%   K1:         intrinsics matrix of the first camera (literature convention)
%   R1:         rotation matrix of the extrinsics of the first camera in the
%               world frame (literature convention)
%   t1:         translation vector of the extrinsics of the first camera in the
%               world frame (literature convention)
%   K2:         intrinsics matrix of the second camera (literature convention)
%   R2:         rotation matrix of the extrinsics of the second camera in the
%               world frame (literature convention)
%   t2:         translation vector of the extrinsics of the second camera in the
%               world frame (literature convention)
%   dir:        name of the directory where to save the results
%
%   Output arguments:
%   ------------------
%   delta_R:    rotation matrix of the extrinsics of the stereo pair with the 
%               first camera as reference (literature convention)
%   delta_t:    translation vector of the extrinsics of the stereo pair with the 
%               first camera as reference (literature convention)
%   E:          essential matrix of the stereo pair (literature convention)
%   F:          fundamental matrix of the stereo pair (literature convention)
%
%   See also CALIBRATION_INTRINSICS_CAMERA, CALIBRATION_EXTRINSICS_CAMERA

    fprintf('----- Stereo Calibration (Extrinsics) -----\n'); 
    
    fprintf('%s\n', dir);    
    
    if ~exist(dir, 'dir')
        mkdir(dir)
    end

    % Find the relative pose of the 2nd camera wrt the 1st camera
    delta_R = R2*R1';
    delta_t = -R2*(R1')*t1 + t2;

    % Calculate E
    E = skew(delta_t) * delta_R;

    % Calculate F
    F = inv(K2)' * E * inv(K1);
    
    % Save results
    save(fullfile(dir, 'delta_R'), 'delta_R');
    save(fullfile(dir, 'delta_t'), 'delta_t');
    save(fullfile(dir, 'E'), 'E');
    save(fullfile(dir, 'F'), 'F');
    
end