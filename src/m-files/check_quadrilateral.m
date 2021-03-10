function is_cvx_quad = check_quadrilateral(points, varargin)
    % points contain the vertices of a 2D figure
    % points = [ [x1,y1]; [x2,y2]; ...; [xN,yN] ]
    
    % Default values of parameters
    default_sum_angles_tolerance  = 10; % degrees
    default_parallelism_tolerance = 15; % degrees
    
    % Input parser
    p = inputParser;
    addParameter(p, 'sum_angles_tolerance', default_sum_angles_tolerance);
    addParameter(p, 'parallelism_tolerance', default_parallelism_tolerance);
    parse(p, varargin{:});
    
    % Parse function parameters
    SUM_ANGLES_TOLERANCE  = p.Results.sum_angles_tolerance;
    PARALLELISM_TOLERANCE = p.Results.parallelism_tolerance;
    
    is_cvx_quad = 0;
    
    % Check if the points define a quadrilateral
    if size(points,1) == 4
        
        % Calculate the vectors from each vertex to the next one
        % and put them in a matrix arranged by column
        V = (circshift(points,-1,1) - points)';
        V_normalized = V ./ vecnorm(V);
        
        % Calculate the internal angles of the quadrilateral
        angles = pi - acos( dot(circshift(V_normalized,-1,2), V_normalized) );
        
        % Calculate the total internal angle of the quadrilateral
        sum_angles = sum(angles);
        sum_angles_degree = sum_angles / pi * 180;
        
        % Check if the points define a convex quadrilateral
        if sum_angles_degree > 360-SUM_ANGLES_TOLERANCE && ...
           sum_angles_degree < 360+SUM_ANGLES_TOLERANCE
            
            % Calculate the angles between opposite sides
            parallel_sides_angles = acos( abs( dot(circshift(V_normalized,-2,2), V_normalized) ) );
            parallel_sides_angles_degree = parallel_sides_angles ./ pi * 180;
            
            % Check if the points define a quasi-parallelogram
            if parallel_sides_angles_degree(1) < PARALLELISM_TOLERANCE && ...
               parallel_sides_angles_degree(2) < PARALLELISM_TOLERANCE
                is_cvx_quad = 1;
            end
            
            % OLD IMPLEMENTATION, JUST AS BACKUP
            % V_old = zeros(4,2);
            % angles_old = zeros(4,1);
            % parallel_sides_angles_old = zeros(2,1);
            % for i=1:4
            %     V_old(i,:) = points(mod(i,4)+1,:) - points(i,:);
            % end
            % for i=1:4
            %     angles_old(i) = pi - acos(...
            %         V_old(mod(i,4)+1,:) * V_old(i,:)' ...
            %         ./ norm(V_old(mod(i,4)+1,:)) ./ norm( V_old(i,:)) ...
            %     );
            % end
            % for i=1:2
            %     parallel_sides_angles_old(i) = acos(...
            %         abs( V_old(i+2,:) * V_old(i,:)' ) ...
            %         ./ norm(V_old(i+2,:)) ./ norm( V_old(i,:)) ...
            %     ); 
            % end
            
        end
        
    end

end