function [R, t] = aruco_pnp(roi, roi_world, K, varargin)
    
%     if nargin == 3
%         verbose = 0;
%     else

    % Default values of parameters
    default_verbose = 1;

    % Input parser
    p = inputParser;
    addParameter(p, 'verbose', default_verbose);
    parse(p, varargin{:});
    
    % Parse function parameters
    VERBOSE = p.Results.verbose;

    % PnP
    [R, t] = exterior_lin(roi', roi_world', K);
    [R, t] = exterior_nonlin(R, t, roi', roi_world', K);
    
    if VERBOSE == 1
        
        %
        
    end

end