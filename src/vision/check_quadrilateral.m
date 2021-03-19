function is_cvx_quad = check_quadrilateral(points, varargin)
% CHECK_QUADRILATERAL Refinement of Perspective-n-Points (PnP) from 3D-2D correspondences.
%   The points contain the vertices of a 2D figure
%   points = [ [x1,y1]; [x2,y2]; ...; [xN,yN] ]
%
%   is_cvx_quad = CHECK_QUADRILATERAL(points, vargin)
%
%   Input arguments:
%   ------------------
%   points:     TODO
%   vargin
%
%   Parameters:
%   ------------------
%   'sum_angles_tol':   TODO
%   'parallelism_tol':  TODO
%   'side_th_low':      TODO
%   'side_th_high':     TODO
%
%   Output arguments:
%   ------------------
%   is_cvx_quad:        return 1 if this is a valid quadrilateral 0 otherwise
    
    % Default values of parameters
    default_sum_angles_tol = 10;
    default_parallelism_tol = 15;
    default_side_th_low = 10;
    default_side_th_high = 700;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'sum_angles_tol', default_sum_angles_tol);
    addParameter(p, 'parallelism_tol', default_parallelism_tol);
    addParameter(p, 'side_th_low', default_side_th_low);
    addParameter(p, 'side_th_high', default_side_th_high);
    parse(p, varargin{:});
    
    % Parse function parameters
    SUM_ANGLES_TOL = p.Results.sum_angles_tol;
    PARALLELISM_TOL = p.Results.parallelism_tol;
    SIDE_TH_LOW = p.Results.side_th_low;
    SIDE_TH_HIGH = p.Results.side_th_high;

    is_cvx_quad = 0;
    
    % Check if the points define a quadrilateral
    if size(points,1) == 4
        
        % Calculate the vectors from each vertex to the next one
        % and put them in a matrix arranged by column
        V = (circshift(points,-1,1) - points)';
        V_norm = vecnorm(V);
        V_normalized = V ./ V_norm;
        
        % Calculate the internal angles of the quadrilateral
        angles = pi - acos( dot(circshift(V_normalized,-1,2), V_normalized) );
        
        % Calculate the total internal angle of the quadrilateral
        sum_angles = sum(angles);
        sum_angles_degree = sum_angles / pi * 180;
        
        % Check if the points define a convex quadrilateral
        if sum_angles_degree > 360-SUM_ANGLES_TOL && ...
           sum_angles_degree < 360+SUM_ANGLES_TOL
            
            % Calculate the angles between opposite sides
            parallel_sides_angles = acos( abs( dot(circshift(V_normalized,-2,2), V_normalized) ) );
            parallel_sides_angles_degree = parallel_sides_angles ./ pi * 180;
            
            % Check if the points define a quasi-parallelogram
            if parallel_sides_angles_degree(1) < PARALLELISM_TOL && ...
               parallel_sides_angles_degree(2) < PARALLELISM_TOL
            
                % Check if the points define very small or very large sides
                if all(V_norm > SIDE_TH_LOW) && all(V_norm < SIDE_TH_HIGH)
           
                    is_cvx_quad = 1;

                end
            
            end
            
        end
        
    end

end