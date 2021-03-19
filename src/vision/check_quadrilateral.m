function is_cvx_quad = check_quadrilateral(points, SUM_ANGLES_TOL, PARALLELISM_TOL, SIDE_TH_LOW, SIDE_TH_HIGH)
    % points contain the vertices of a 2D figure
    % points = [ [x1,y1]; [x2,y2]; ...; [xN,yN] ]
    
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