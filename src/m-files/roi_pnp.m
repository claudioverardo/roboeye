function [R, t] = roi_pnp(roi, roi_world, K) 

    [R, t] = exterior_lin(roi', roi_world', K);
    [R, t] = exterior_nonlin(R, t, roi', roi_world', K);
    
end