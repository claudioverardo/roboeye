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