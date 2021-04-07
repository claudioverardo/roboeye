function is_valid_quad = check_quadrilateral(points, varargin)
% CHECK_QUADRILATERAL Check if the set of input points defines the shape of
% a valid quadrilateral, i.e., it is close to the shape of a parallelogram.
%
%   is_valid_quad = CHECK_QUADRILATERAL(points)
%
%   Input arguments:
%   ------------------
%   points:             array Nx2 of points that defines the shape
%                       [ [x1,y1]; [x2,y2]; ... ; [xN,yN] ]
%
%   Parameters:
%   ------------------
%   'sum_angles_tol':   tolerance on the sum of the internal angles [degrees]
%   'parallelism_tol':  tolerance on the angle between opposite sides [degrees]
%   'side_th_low':      lower threshold on the length of each side [pixels]
%   'side_th_high':     higher threshold on the length of each side [pixels]
%   'angle_th_low':     lower threshold on the internal angles [degrees]
%   'angle_th_high':    higher threshold on the internal angles [degrees]
%
%   Output arguments:
%   ------------------
%   is_valid_quad:      1 if the shape is a valid quadrilateral 0 otherwise
%
%   NOTE: a shape is discarded when one of the following conditions is met:
%   - sum of the internal angles > 360° + sum_angles_tol 
%   - angle between opposide sides > parallelism_tol
%   - length of a side < side_th_low
%   - length of a side > side_th_high
%   - value of an internal < angle_th_low
%   - value of an internal > angle_th_high
%
%   See also ROI_REFINEMENT
    
    % Default values of parameters
    default_sum_angles_tol = 10;
    default_parallelism_tol = 15;
    default_side_th_low = 10;
    default_side_th_high = 700;
    default_angle_th_low = 20;
    default_angle_th_high = 160;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'sum_angles_tol', default_sum_angles_tol);
    addParameter(p, 'parallelism_tol', default_parallelism_tol);
    addParameter(p, 'side_th_low', default_side_th_low);
    addParameter(p, 'side_th_high', default_side_th_high);
    addParameter(p, 'angle_th_low', default_angle_th_low);
    addParameter(p, 'angle_th_high', default_angle_th_high);
    parse(p, varargin{:});
    
    % Parse function parameters
    SUM_ANGLES_TOL = p.Results.sum_angles_tol;
    PARALLELISM_TOL = p.Results.parallelism_tol;
    SIDE_TH_LOW = p.Results.side_th_low;
    SIDE_TH_HIGH = p.Results.side_th_high;
    ANGLE_TH_LOW = p.Results.angle_th_low;
    ANGLE_TH_HIGH = p.Results.angle_th_high;

    is_valid_quad = 0;
    
    % Check if the points define a quadrilateral
    if size(points,1) == 4
        
        % Calculate the vectors from each vertex to the next one
        % and put them in a matrix arranged by column
        V = (circshift(points,-1,1) - points)';
        V_norm = vecnorm(V);
        V_normalized = V ./ V_norm;
        
        % Calculate the internal angles of the quadrilateral
        angles = pi - acos( dot(circshift(V_normalized,-1,2), V_normalized) );
        angles_degrees = angles / pi * 180;
        
        % Calculate the total internal angle of the quadrilateral
        % sum_angles = sum(angles);
        sum_angles_degree = sum(angles_degrees);
        
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
            
                    % Check if the points define very small or very large angles
                    if all(angles_degrees > ANGLE_TH_LOW) && all(angles_degrees < ANGLE_TH_HIGH)

                        is_valid_quad = 1;

                    end

                end
            
            end
            
        end
        
    end

end