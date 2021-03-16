function [R, t] = pnp(X_image, X_world, K_obj)
% Perspective-n-points (PnP)
% X_image: Mx2 array
% X_world: Mx3 array

    % Compute the rototranslation via PnP [Fusiello]
    % [R, t] = exterior_lin(X_image', X_world', K_obj.IntrinsicMatrix');
    % [R, t] = exterior_nonlin(R, t, X_image', X_world', K_obj.IntrinsicMatrix');
    % R = R';
    % t = t';
    
    % Compute the rototranslation via PnP [Matlab]
    % NOTE: extrinsics() assume the X_world points with z=0
    [R, t] = extrinsics(X_image, X_world(:,1:2), K_obj);
    % [R_camera, t_camera] = estimateWorldCameraPose(X_image, X_world, K_obj);
    % [R, t] = cameraPoseToExtrinsics(R_camera, t_camera);

    
end
