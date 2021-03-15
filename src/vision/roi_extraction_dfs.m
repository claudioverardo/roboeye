function components = roi_extraction_dfs(img_canny_local)
    
    VERBOSE = 0;
    
    % Edge detection image
    global img_canny;
    img_canny = img_canny_local;
    
    % Memory for the dfs process (visited)
    global visited;
    visited = imbinarize(zeros(size(img_canny)));

    % Connected components image result
    global img_result;
    img_result = zeros([size(img_canny) 3]);
    img_result = cast(img_result, 'uint8');
    
    % Raw ROIs, tails
    global components;
    components = cell(0, 2);
    
    for i = 1:size(img_canny,1)
        for j = 1:size(img_canny,2)
            if (visited(i, j) == 0 && ...
                img_canny(i, j) == 1) 
                % Generate random color fot the connected component
                color = [rand(1), rand(1), rand(1)] * 255;
                
                % Create new empty connected component (points, tails, polyfit)
                components{size(components, 1) + 1, 1} = [];
                components{size(components, 1), 2} = [];
                
                % Explore that component
                % [tmp_components, tmp_tails] = ...
                %   bfs_c(img_canny, visited, size(img_canny, 1), size(img_canny, 2), i, j);
                dfs(i, j, color); % Matlab implementation
                
                % components{size(components, 1), 1} = [components{size(components, 1), 1}; tmp_components];
                % components{size(components, 1), 2} = [components{size(components, 1), 2}; tmp_tails];
                
                for idx=1:size(components{size(components, 1), 1}, 1)
                    visited(components{size(components, 1), 1}(idx, 2), components{size(components, 1), 1}(idx, 1)) = 1;
                end
                
                % figure;
                % imshow(img_canny);
                % hold on;
                % plot(comp(:, 1), comp(:, 2), "ro");
                % 
                % figure;
                % imshow(visited);
                % hold on;
                % plot(comp(:, 1), comp(:, 2), "ro");
                
                % Add to components tails the startpoint if there is a tail
                % if (check_tail(i, j) == 1)
                %     components{size(components, 1), 2} = [components{size(components, 1), 2}; [j, i]];
                % end
                % 
                % % Check if component is not closed and delete it (minimum 100 pixels)
                % invalid_component = 0;
                % component = cell(0, 2);
                % component{1, 1} = components{size(components, 1), 1};
                % component{1, 2} = components{size(components, 1), 2};
                % if (check_connected_component(component) == 0)
                %     components(size(components, 1), :) = [];
                %     invalid_component = 1;
                % end
                % 
                % % If component is valid, apply polyfit
                % if (invalid_component == 0)
                %     components{size(components, 1), 1}(end+1,:) = components{size(components, 1), 1}(1,:);
                % end
            end
        end
    end
    
    if VERBOSE > 0
        I = zeros(size(img_result));
        for i = 1:size(components, 1)
            % plot component points
            for j = 1:size(components{i, 1}, 1)
                I(components{i, 1}(j, 2), components{i, 1}(j, 1), :) = [255, 255, 255];
            end
            % plot tails
            for j = 1:size(components{i, 2}, 1)
                I(components{i, 2}(j, 2), components{i, 2}(j, 1), :) = [255, 0, 0];
            end
        end
        % I = [img, img_result, I];
        figure;
        imshow(img_result);
        title('Components');
    end
    
end


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
        
        if (check_boundaries(v_i, v_j, size(img_canny)) == 1 && ...
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


function ans = check_tail(pixel_i, pixel_j)   
    global img_canny;

    filled = 0;
    
    for i = -1:1
        for j = -1:1
            if (check_boundaries(pixel_i + i, pixel_j + j, size(img_canny)) == 1 && ...
                img_canny(pixel_i + i, pixel_j + j) ~= 0)
                filled = filled + 1;
            end
        end
    end

    if (filled < 3)
        ans = 1;
        return;
    elseif (filled == 3)
        isOk = 0;

        % Corner case top left
        partial_filled = 0;
        for i = -1:0
            for j = -1:0
                if (check_boundaries(pixel_i + i, pixel_j + j, size(img_canny)) == 1 && ...
                    img_canny(pixel_i + i, pixel_j + j) ~= 0)
                    partial_filled = partial_filled + 1;
                end
            end
        end
        if (partial_filled == 3) 
            isOk = 1;
        end

         % Corner case top right
        partial_filled = 0;
        for i = -1:0
            for j = 0:1
                if (check_boundaries(pixel_i + i, pixel_j + j, size(img_canny)) == 1 && ...
                    img_canny(pixel_i + i, pixel_j + j) ~= 0)
                    partial_filled = partial_filled + 1;
                end
            end
        end
        if (partial_filled == 3) 
            isOk = 1;
        end

         % Corner case bottom left
        partial_filled = 0;
        for i = 0:1
            for j = -1:0
                if (check_boundaries(pixel_i + i, pixel_j + j, size(img_canny)) == 1 && ...
                    img_canny(pixel_i + i, pixel_j + j) ~= 0)
                    partial_filled = partial_filled + 1;
                end
            end
        end
        if (partial_filled == 3) 
            isOk = 1;
        end

         % Corner case bottom right
        partial_filled = 0;
        for i = 0:1
            for j = 0:1
                if (check_boundaries(pixel_i + i, pixel_j + j, size(img_canny)) == 1 && ...
                    img_canny(pixel_i + i, pixel_j + j) ~= 0)
                    partial_filled = partial_filled + 1;
                end
            end
        end
        if (partial_filled == 3) 
            isOk = 1;
        end

        % Corner case perfect L 
        if (check_boundaries(pixel_i - 1, pixel_j, size(img_canny)) == 1 && ...
            check_boundaries(pixel_i, pixel_j - 1, size(img_canny)) == 1 && ...
            img_canny(pixel_i - 1, pixel_j) ~= 0 && ...
            img_canny(pixel_i, pixel_j - 1) ~= 0)
            isOk = 0;
        end 

        % Corner case perfect L 
        if (check_boundaries(pixel_i - 1, pixel_j, size(img_canny)) == 1 && ...
            check_boundaries(pixel_i, pixel_j + 1, size(img_canny)) == 1 && ...
            img_canny(pixel_i - 1, pixel_j) ~= 0 && ...
            img_canny(pixel_i, pixel_j + 1) ~= 0)
            isOk = 0;
        end 

        % Corner case perfect L 
        if (check_boundaries(pixel_i + 1, pixel_j, size(img_canny)) == 1 && ...
            check_boundaries(pixel_i, pixel_j - 1, size(img_canny)) == 1 && ...
            img_canny(pixel_i + 1, pixel_j) ~= 0 && ...
            img_canny(pixel_i, pixel_j - 1) ~= 0)
            isOk = 0;
        end 

        % Corner case perfect L 
        if (check_boundaries(pixel_i + 1, pixel_j, size(img_canny)) == 1 && ...
            check_boundaries(pixel_i, pixel_j + 1, size(img_canny)) == 1 && ...
            img_canny(pixel_i + 1, pixel_j) ~= 0 && ...
            img_canny(pixel_i, pixel_j + 1) ~= 0)
            isOk = 0;
        end 

        if isOk == 1
            ans = 1;
            return;
        end
    end
    
    ans = 0;
    return;
end


function ans = check_connected_component(component)
%     image = zeros(1080, 1920);
    
%     for j = 1:size(component{1, 1}, 1)
%         image(component{1, 1}(j, 2), component{1, 1}(j, 1)) = 255;
%     end
%     image = cast(image, 'uint8');
%     imshow(image);
%     hold on;
    
    if (size(component{1, 2}, 1) > 2 || ...     % Tails Threshold
        size(component{1, 2}, 1) == 1 || ...    % Tails Threeshold
        size(component{1, 1}, 1) < 35 || ...    % Minumum Pixel Threeshold
        ( ...
            size(component{1, 2}, 1) == 2 && ...
            sum(abs(component{1, 2}(1, :)' - component{1, 2}(2, :)')) > 8 ...   % Manhattan distance Threeshold
        ))
    
           ans = 0;
           return;
        % Check distance beetween tails and point of the connected component
%         for i = 1:size(component{1, 2}, 1)
%             manhattan_distance = inf;
%             for j = 1:size(component{1, 1}, 1)
%                 tail = component{1, 2}(i, :);
%                 point = component{1, 1}(j, :);
%                 manhattan_distance = sum(abs(component{1, 2}(1, :)' - component{1, 2}(2, :)'))
%             end
%             if (manhattan_distance > 8) % Manhattan distance Threeshold
%                 ans = 0;
%                 return;
%             end
%         end
%         
%         return;
    end
    
    ans = 1;
    return;
end
