function Y = hom_tf(X, H, K, k)
% HOM_TF Apply the homogeneous transformation H to the set of points X. 
% The points are arranged by rows X = [x1; ... ; xN] and Y = [y1; ... ; yN].
% The transformation acts on the homogeneous coordinates, hom(Y) = hom(X)*H.
% If required, apply also radial distortion to the results.
%
%   Y = HOM_TF(X, H)
%
%   Y = HOM_TF(X, H, K, k)
%
%   Input arguments:
%   ------------------
%   X:  input set of points (inhomogeneous coordinates)
%   H:  transformation between homogeneous coordinates (Matlab convention)
%       - H 4x3 is a projection 
%       - H 3x3 is a transformation in the projective plane
%       - H 4x4 is a transformation in the projective space
%   K:  intrisics matrix of the camera (optional)
%   k:  radial distortion coefficients of the camera (optional)
%
%   Output arguments:
%   ------------------
%   Y:      transformed set of points (inhomogeneous coordinates)
%
%   NOTE: if intrinsics K and radial distortion coefficients k are provided,
%   the points Y must be 2D image points.

    % Convert X in homogeneous coordinates
    X_hom = [X ones(size(X,1),1)];
    
    % Apply transformation
    Y_hom = X_hom * H; 
    
    % Projective normalization
    Y_hom = Y_hom ./ Y_hom(:, end);

    % Return the result with inhomogeneous coordinates
    Y = Y_hom(:,1:end-1);
    
    % Apply radial distortion (if required)
    % NOTE: here Y must be a set of 2D image points
    if nargin > 2
        Y = rad_dist_apply(Y, K, k);
    end

end
