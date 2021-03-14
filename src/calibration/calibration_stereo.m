function [F, E, deltaT] = calibration_stereo(P1, P2)
    T1 = external_orientation_camera(P1);
    T2 = external_orientation_camera(P2);

    deltaT = cell(1, size(P1, 2));
    for i = 1:size(P1, 2)
        deltaT{i} = T2{i} * inv(T1{i});
    end

    % Evalutate results
    % svd([vec(deltaT{1}), vec(deltaT{2}), vec(deltaT{3}), vec(deltaT{4})])

    % Calculate E and F on the first deltaT
    deltaT1 = deltaT{1};

    % Calculate E
    E = skew(deltaT1(1:3, 4)) * deltaT1(1:3, 1:3);

    % Calculate F
    F = inv(K2)' * E * inv(K1);
end