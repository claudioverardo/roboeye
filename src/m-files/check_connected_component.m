function ans = check_connected_component(component)
    if (size(component{1, 2}, 1) > 2 || ...     % Tails Threshold
        size(component{1, 2}, 1) == 1 || ...    % Tails Threeshold
        size(component{1, 1}, 1) < 35 || ...    % Minumum Pixel Threeshold
        ( ...
            size(component{1, 2}, 1) == 2 && ...
            sum(abs(component{1, 2}(1, :)' - component{1, 2}(2, :)')) > 8 ...   % Manhattan distance Threeshold
        ))
        ans = 0;
        return;
    end
    
    ans = 1;
    return;
end