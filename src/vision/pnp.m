function [R, t] = pnp(X_image, X_world, K_obj)
% PNP Perspective-n-Points (PnP) from 3D-2D correspondences
%
% X_image: Mx2 array
% X_world: Mx3 array
%
% NB: points and K with Matlab convention, X_image = X_world*[R;t]*K
    
    % Compute extrinsics via PnP [Matlab - P3P+MSAC]
    % [R_camera, t_camera] = estimateWorldCameraPose(X_image, X_world, K_obj);
    % [R, t] = cameraPoseToExtrinsics(R_camera, t_camera);
    
    % Compute extrinsics via PnP [Matlab - plane homography resection]
    [R, t] = extrinsics(X_image, X_world(:,1:2), K_obj); % X_world points assumed with z=0
    
    % Non linear refinement
    [R, t] = pnp_nonlin(R, t, X_image, X_world, K_obj);
    
end

% Compute extrinsics via PnP [Fusiello - Fiore+refinement]
% [R, t] = exterior_lin(X_image', X_world', K_obj.IntrinsicMatrix');
% [R, t] = exterior_nonlin(R, t, X_image', X_world', K_obj.IntrinsicMatrix');
% R = R';
% t = t';
