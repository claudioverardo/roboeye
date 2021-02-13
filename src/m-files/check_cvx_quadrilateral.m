function is_cvx_quad = check_cvx_quadrilateral(points)
    
    % points = [ [x1,y1]; [x2,y2]; [x3,y3]; [x4,y4] ] vertices
    
    is_cvx_quad = 0;
    vectors = zeros(4,2);
    angles = zeros(4,1);
    
    if size(points,1) == 4
        for i=1:4
            vectors(i,:) = points(mod(i,4)+1,:) - points(i,:);
        end
        
        for i=1:4
            angles(i) = pi - acos(...
                vectors(mod(i,4)+1,:) * vectors(i,:)' ...
                ./ norm(vectors(mod(i,4)+1,:)) ./ norm( vectors(i,:)) ...
            );
        end
        
        tot_angle = sum(angles);
        
        if tot_angle > 0.97*2*pi && tot_angle < 1.03*2*pi
            
            parallel_sides_angles = zeros(2,1);
            for i=1:2
                parallel_sides_angles(i) = acos(...
                    abs( vectors(i+2,:) * vectors(i,:)' ) ...
                    ./ norm(vectors(i+2,:)) ./ norm( vectors(i,:)) ...
                ); 
            end
                
            if parallel_sides_angles(1) < pi/10 && parallel_sides_angles(2) < pi/10
                is_cvx_quad = 1;
            end
            
        end
        
    end

end