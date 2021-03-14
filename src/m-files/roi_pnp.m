function [R, t] = roi_pnp(roi, roi_world, K_obj)

    % NB: points and K with Matlab convention

    % Compute the rototranslation via PnP [Fusiello]
    % [R, t] = exterior_lin(roi', roi_world', K_obj.IntrinsicMatrix');
    % [R, t] = exterior_nonlin(R, t, roi', roi_world', K_obj.IntrinsicMatrix');
    % R = R';
    % t = t';
    
    % Compute the rototranslation via PnP [Matlab]
    [R, t] = extrinsics(roi, roi_world(:,1:2), K_obj);
    
end