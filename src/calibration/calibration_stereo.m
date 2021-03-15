function [F, E, deltaT] = calibration_stereo(P1, K1, P2, K2)

    fprintf('-------- Stereo Calibration --------\n');

    % Calculate the T extrinsic matrices
    T1 = extrinsics_camera(P1, K1);
    T2 = extrinsics_camera(P2, K2);
    
    num_cameras = length(P1);

    deltaTs = cell(1, num_cameras);
    for i = 1:num_cameras
        deltaTs{i} = T2{i} * inv(T1{i});
    end
    
    % Test validity of results
    fprintf('test svd\n');
    sigma_svd = check_svd(deltaTs);
    for i = 1:length(sigma_svd)
        fprintf('sigma %2d: %f\n', i, sigma_svd(i));
    end

    % Calculate E and F on the first deltaT
    deltaT = deltaTs{1};

    % Calculate E
    E = skew(deltaT(1:3, 4)) * deltaT(1:3, 1:3);

    % Calculate F
    F = inv(K2)' * E * inv(K1);
    
end