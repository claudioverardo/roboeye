function [R,t,G] = get_extrinsics_camera(P, K) 
% GET_EXTRINSICS_CAMERA Retrieve the extrinsics of a set of cameras from
% their projection matrices and intrinsics matrices.
%
%   [R,t,G] = GET_EXTRINSICS_CAMERA(P, K) 
%
%   Input arguments:
%   ------------------
%   P: cell array of projection matrices (literature convention)
%   K: cell array of intrinsics matrices (literature convention)
%
%   Output arguments:
%   ------------------
%   R: cell array of rotation matrices (literature convention)
%   t: cell array of translation vectors (literature convention)
%   G: cell array of [R t; 0 1] matrices (literature convention)

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