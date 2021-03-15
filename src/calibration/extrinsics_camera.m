function T = extrinsics_camera(P, K) 
% Retrieve camera pose T from projective matrix P and intrinsics K
    num_cameras = length(P);
    T = cell(1, num_cameras);
    for i = 1:num_cameras
        T{i} = [K\P{i}; [0, 0, 0, 1]];
    end
end