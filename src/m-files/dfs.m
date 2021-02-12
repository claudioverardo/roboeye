function dfs(u_i, u_j, color)

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
            dfs(v_i, v_j, color);
        end
    end
    
    if possible_tail == 1
        % Check boundaries conditions
        if (check_tail(u_i, u_j) == 1)
            components{size(components, 1), 2} = [components{size(components, 1), 2}; [u_j, u_i]];
        end
    end
end