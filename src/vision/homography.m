function Y = homography(X, H)
% HOMOGRAPHY Apply the homogeneous transformation H to the set of points X. 
% The points are arranged by rows X = [x1; ... ; xN] and Y = [y1; ... ; yN].
% The transformation acts on the homogeneous coordinates, hom(Y) = hom(X)*H.
%
%   Y = HOMOGRAPHY(X, H)
%
%   Input arguments:
%   ------------------
%   X:      input set of points (inhomogeneous coordinates)
%   H:      linear transformation between homogeneous coordinates
%           (Matlab convention)
%
%   Output arguments:
%   ------------------
%   Y:      transformed set of points (inhomogeneous coordinates)
%
%   NOTE: the kind of transformation depends on the dimensions of H
%         - H 4x3 is a projection 
%         - H 3x3 is a transformation in the projective plane
%         - H 4x4 is a transformation in the projective space
    
    % Convert X in homogeneous coordinates
    X_hom = [X ones(size(X,1),1)];
    
    % Apply transformation
    Y_hom = X_hom * H; 
    
    % Projective normalization
    Y_hom = Y_hom ./ Y_hom(:, end);

    % Return the result with inhomogeneous coordinates
    Y = Y_hom(:,1:end-1);

end

