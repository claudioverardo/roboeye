function components = roi_extraction(VERBOSE)

    global img_canny;
    global visited;
    global img_result;
    global components;
    components = cell(0, 2);
    
    for i = 1:size(img_canny,1)
        for j = 1:size(img_canny,2)
            if (visited(i, j) == 0 && ....
                img_canny(i, j) == 1) 
                % Generate random color fot the connected component
                color = [rand(1), rand(1), rand(1)] * 255;
                
                % Create new empty connected component (points, tails, polyfit)
                components{size(components, 1) + 1, 1} = [];
                components{size(components, 1), 2} = [];
                
                % Explore that component
                [tmp_components, tmp_tails] = ...
                   bfs_c(img_canny, visited, size(img_canny, 1), size(img_canny, 2), i, j);
                % dfs(i, j, color); % Matlab implementation
                
                components{size(components, 1), 1} = [components{size(components, 1), 1}; tmp_components];
                components{size(components, 1), 2} = [components{size(components, 1), 2}; tmp_tails];
                
                for idx=1:size(components{size(components, 1), 1}, 1)
                    visited(components{size(components, 1), 1}(idx, 2), components{size(components, 1), 1}(idx, 1)) = 1;
                end
                
%                 figure;
%                 imshow(img_canny);
%                 hold on;
%                 plot(comp(:, 1), comp(:, 2), "ro");
%                 
%                 figure;
%                 imshow(visited);
%                 hold on;
%                 plot(comp(:, 1), comp(:, 2), "ro");
                
                % Add to components tails the startpoint if there is a tail
                if (check_tail(i, j) == 1)
                    components{size(components, 1), 2} = [components{size(components, 1), 2}; [j, i]];
                end
                
                % Check if component is not closed and delete it (minimum 100 pixels)
                invalid_component = 0;
                component = cell(0, 2);
                component{1, 1} = components{size(components, 1), 1};
                component{1, 2} = components{size(components, 1), 2};
                if (check_connected_component(component) == 0)
                    components(size(components, 1), :) = [];
                    invalid_component = 1;
                end
                
                % If component is valid, apply polyfit
                if (invalid_component == 0)
                    components{size(components, 1), 1}(end+1,:) = components{size(components, 1), 1}(1,:);
                end
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
        imshow(I);
        title('ROI extraction');
    end
    
end