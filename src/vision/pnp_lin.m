function [R, t, reproj_err] = pnp_lin(X_image, X_world, K)
% PNP Perspective-n-Points (PnP) from 3D-2D correspondences.
%   
%   [R, t] = PNP_LIN(X_image, X_world, K) finds the camera pose R, t from a
%   set of 2D-3D correspondences defined by X_image, X_world respectively.
%   The algorithm assumes 3D points with z=0, i.e. X_world(:,3)=0.
%   
%   [R, t, reproj_err] = PNP_LIN(X_image, X_world, K) return also the RMS
%   value of the reprojection errors of the 3D-2D correspondences.
%
%   Input arguments:
%   ------------------
%   X_image:    Nx2 array
%   X_world:    Nx3 array
%
%   Output arguments:
%   ------------------
%   R:          rotation matrix 3x3
%   t:          translate vector 3x1
%   reproj_err: reprojection error (RMS value)
%   
%   NOTE: points and K with Matlab conventions, X_image = X_world*[R;t]*K.
%
%   See also PNP_NONLIN, REPROJECTION_ERROR

    % Compute plane homography between X_image and X_world
    % NOTE: X_world points assumed with z=0
    H_obj = fitgeotrans(X_world(:,1:2), X_image, 'projective');
    
    % Switching to computer vision literature conventions
    H = H_obj.T'; 
    K = K'; 
    
    % H represents the projective transformation between 2 planes
    % x = H*X = lambda*K*[r1 r2 t]*X
    r1_r2_t = K \ H;
    
    % Normalization factor (it ensures norm(r1)=1)
    lambda = 1 / norm(r1_r2_t(:, 1));
    r1_r2_t = r1_r2_t * lambda;
    
    % Extract R and t from H
    r1 = r1_r2_t(:,1);
    r2 = r1_r2_t(:,2);
    r3 = cross(r1, r2);
    R = [r1 r2 r3];
    t = r1_r2_t(:,3);
    
    % If the center of projection is in the negative halfplane ...
    COP = -R'*t;
    if [0,0,1]*COP < 0
        % ... change sign and re-extract R,t
        r1_r2_t = -r1_r2_t;
        r1 = r1_r2_t(:,1);
        r2 = r1_r2_t(:,2);
        r3 = cross(r1, r2);
        R = [r1 r2 r3];
        t = r1_r2_t(:,3);
    end

    % Project the matix R onto SO(3) to obtain a valid rotation
    [U, ~, V] = svd(R);
    R = U*diag([1,1,det(V'*U)])*V';
    
    % Back to Matlab conventions
    K = K';
    R = R';
    t = t';
    
    % Compute the reprojection error of each 2D-3D correspondence
    n_points = size(X_world,1);
    reproj_errs = zeros(n_points,1);
    for i=1:n_points
        reproj_errs(2*i-1:2*i) = reprojection_error(X_image(i,:), X_world(i,:), K, R, t);
    end
    
    % Return the overall reprojection error (RMS value)
    reproj_err = rms(reproj_errs);

    % PnP (Fiore algorithm) with Fusiello Computer Vision Toolkit [DEBUG]
    % [R, t] = exterior_lin(X_image', X_world', K');
    % R = R';
    % t = t'; 
end
