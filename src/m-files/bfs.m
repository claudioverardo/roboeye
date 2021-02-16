function bfs(start_i, start_j, color)
    global img_canny;
    global visited;
    global img_result;
    global components;

    % by row, from left
    % add_i = [-1, -1, -1,  0,  0,  1,  1,  1];
    % add_j = [-1,  0, +1, -1,  1, -1,  0,  1];
    
    % clockwise, from top-left
    % add_i = [-1, -1, -1,  0, +1, +1, +1,  0];
    % add_j = [-1,  0, +1, +1, +1,  0, -1, -1];
    
    % clockwise, from right
    % add_i = [ 0, +1, +1, +1,  0, -1, -1, -1];
    % add_j = [+1, +1,  0, -1, -1, -1,  0, +1];
    
    % crosses and then diagonals
    add_i = [ 0, +1,  0, -1, -1, +1, +1, -1];
    add_j = [+1,  0, -1,  0, +1, +1, -1, -1];
    
    % Create queue and push start node inside it
    queue_physical_size = size(img_result, 1) * size (img_result, 2) / 16;
    queue_logical_size = 1;
    queue = zeros(queue_physical_size, 2); 
    queue(end, :) = [start_i, start_j];
    
    while (queue_logical_size > 0) 
        % Get first node from the queue
        u_i = queue(queue_physical_size - queue_logical_size + 1, 1);
        u_j = queue(queue_physical_size - queue_logical_size + 1, 2);
        
        % Erase element from the queue
        queue_logical_size = queue_logical_size - 1;
        
        % This node isn't visited so I'm going to visit it
        visited(u_i, u_j) = 1;
        img_result(u_i, u_j, :) = color;
        components{size(components, 1), 1} = [components{size(components, 1), 1}; [u_j, u_i]];
        
        possible_tail = 1;
        
        for idx = 1:8
            % Calculate new neighbord pixel coords
            v_i = u_i + add_i(idx);
            v_j = u_j + add_j(idx);

            if (check_boundaries(v_i, v_j) == 1 && ...
                visited(v_i, v_j) == 0 && ...
                    img_canny(v_i, v_j) == 1)
                possible_tail = 0;
                
                % Push inside the queue
                queue(queue_physical_size - queue_logical_size, :) = [v_i, v_j];
                queue_logical_size = queue_logical_size + 1;
            end
        end
        
        if possible_tail == 1
            % Check boundaries conditions
            if (check_tail(u_i, u_j) == 1)
                components{size(components, 1), 2} = [components{size(components, 1), 2}; [u_j, u_i]];
            end
        end
        
    end
end