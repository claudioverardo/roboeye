function [R,t,G] = get_extrinsics_camera(P, K) 
% Retrieve camera pose T from projective matrix P and intrinsics K

    n_cameras = numel(P);
    
    R = cell(1, n_cameras);
    t = cell(1, n_cameras);
    for i = 1:n_cameras
        Rt = K\P{i};
        R{i} = Rt(1:3,1:3);
        t{i} = Rt(1:3,4);
    end
    
    if nargout > 2
        G = cell(1, n_cameras);
        for i = 1:n_cameras
            G{i} = [R{i} t{i}; 0 0 0 1];
        end
    end
    
end