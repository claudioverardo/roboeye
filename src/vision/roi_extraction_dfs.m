function components = roi_extraction_dfs(img_canny)
% ROI_EXTRACTION_DFS Extract the connected components (set of points, set of
%   tails) from the image (2D-Graph)
%
%   Y = ROI_EXTRACTION_DFS(img_canny) apply the DFS algorithm on the
%   picture and extract the connected components
%
%   The outpur cell array components are arranged in a cell array where each 
%   cell is composed by two elements: the first one is the set of points 
%   that rappresents the component awnd the second one is the of tails for 
%   that component
%   
%   Input arguments:
%   ------------------
%   img_canny:      HEIGHT x WIDTH matrix fashion
%
%   Output arguments:
%   ------------------
%   components:     cell array of the connected components (points and tails)
%
%   See also ROI_EXTRACTION, ROI_EXTRACTION_DFS_C
    
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
                
                % Create new empty connected component (points, tails)
                components{size(components, 1) + 1, 1} = [];
                components{size(components, 1), 2} = [];
                
                % Explore that component through the DFS algoritm
                dfs(img_canny, i, j, color); 
            end
        end
    end    
end

function dfs(img_canny, u_i, u_j, color)

    global visited;
    global img_result;
    global components;
    
    % crosses and then diagonals
    add_i = [ 0, +1,  0, -1, -1, +1, +1, -1];
    add_j = [+1,  0, -1,  0, +1, +1, -1, -1];
    
    % This node isn't visited so I'm going to visit it
    visited(u_i, u_j) = 1;
    img_result(u_i, u_j, :) = color;
    
    % Push in the lastest new component this point
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
            dfs(img_canny, v_i, v_j, color);
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
