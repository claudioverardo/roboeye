function [R, t] = roi_pnp(roi, roi_world, K)

    % NB: points and K with Matlab convention

    % Compute the rototranslation via PnP [Fusiello]
    % [R, t] = exterior_lin(roi', roi_world', K');
    % [R, t] = exterior_nonlin(R, t, roi', roi_world', K');
    % R = R';
    % t = t';
    
    % Compute the rototranslation via PnP [Matlab]
    focal_length    = [K(1,1) K(2,2)]; 
    principal_point = [K(3,1) K(3,2)];
    image_size      = [1920, 1080];
    skew            = K(2,1);
    K_obj = cameraIntrinsics(focal_length, principal_point, image_size, 'Skew', skew);
    [R, t] = extrinsics(roi, roi_world(:,1:2), K_obj);
    
end