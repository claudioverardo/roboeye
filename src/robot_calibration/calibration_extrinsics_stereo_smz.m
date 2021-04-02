function [delta_R, delta_t, E, F] = calibration_extrinsics_stereo_smz(P1, K1, P2, K2, dir)
% CALIBRATION_EXTRINSICS_STEREO_SMZ Retrieve the extrinsics and epipolar matrices
% of a stereo pair. The two cameras are assumed to be previously jointly calibrated
% with the SMZ algorithm and with fixed relative position afterwards.
%
%   [delta_R, delta_t, E, F] = CALIBRATION_EXTRINSICS_STEREO_SMZ(P1, K1, P2, K2, dir)
%
%   Input arguments:
%   ------------------
%   P1:         cell array of projection matrices returned by SMZ calibration
%               of the first camera (literature convention)
%   K1:         intrinsics matrix of the first camera (literature convention)
%   P2:         cell array of projection matrices returned by SMZ calibration
%               of the second camera (literature convention)
%   K2:         intrinsics matrix of the second camera (literature convention)
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
%   See also CALIBRATION_INTRINSICS_CAMERA, CALIBRATION_EXTRINSICS_STEREO

    fprintf('----- Stereo Calibration SMZ (Extrinsics) -----\n');   
    
    fprintf('%s\n', dir); 
    
    if ~exist(dir, 'dir')
        mkdir(dir)
    end

    % Calculate the extrinsics of each camera matrix
    [R1, t1, G1] = get_extrinsics_camera(P1, K1);
    [R2, t2, G2] = get_extrinsics_camera(P2, K2);
    
    n_cameras = numel(P1);

    % Find the relative pose of the 2nd camera wrt the 1st camera
    delta_Gs = cell(1, n_cameras); % roto-translation 3x4
    delta_Rs = cell(1, n_cameras); % rotation 3x3
    delta_ts = cell(1, n_cameras); % translation 3x1
    for i = 1:n_cameras
        delta_Gs{i} = G2{i} * inv(G1{i});
        delta_Rs{i} = delta_Gs{i}(1:3,1:3);
        delta_ts{i} = delta_Gs{i}(1:3,4);
        % delta_Rs{i} = R2{i}*R1{i}';
        % delta_ts{i} = -R2{i}*(R1{i}')*t1{i} + t2{i};
        % delta_Gs{i} = [delta_Rs{i} delta_ts{i}; 0 0 0 1];
    end
    
    % Test validity of results
    fprintf('test svd\n');
    sigma_svd = check_svd(delta_Gs);
    for i = 1:length(sigma_svd)
        fprintf('sigma %2d: %f\n', i, sigma_svd(i));
    end

    % Return the first relative pose as result
    delta_R = delta_Rs{1};
    delta_t = delta_ts{1};

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