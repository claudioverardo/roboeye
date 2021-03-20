function Y = homography(X, H)
% HOMOGRAPHY Apply homogeneous transformation
%
%   Y = HOMOGRAPHY(X, H) apply transformation H to the set of points X 
%   The points are arranged by rows X = [x1; ... ; xN], Y = [y1; ... ; yN], and hom(Y) = hom(X)*H.
%
%   Input arguments:
%   ------------------
%   X:      Input set points (inhomogeneous coordinates)
%   H:      Linear transformation between homogeneous coordinates
%           (Matlab convention: hom(Y) = hom(X)*H)
%
%   Output arguments:
%   ------------------
%   Y:      Transformed output points (Matlab convention)
%
%   NOTE:   H 4x3 is a projection 
%           H 3x3 is a transformation in the projective plane
%           H 4x4 is a transformation in the projective space
    
    % Convert X in homogeneous coordinates
    X_hom = [X ones(size(X,1),1)];
    
    % Apply transformation
    Y_hom = X_hom * H; 
    
    % Projective normalization
    Y_hom = Y_hom ./ Y_hom(:, end);

    % Return the result with inhomogeneous coordinates
    Y = Y_hom(:,1:end-1);

end

