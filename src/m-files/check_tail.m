function ans = check_tail(pixel_i, pixel_j)   
    global img_canny;

    filled = 0;
    
    for i = -2:2
        for j = -2:2
            if (check_boundaries(pixel_i + i, pixel_j + j) == 1 && ...
                img_canny(pixel_i + i, pixel_j + j) ~= 0)
                filled = filled + 1;
            end
        end
    end

    if (filled < 5)
        ans = 1;
        return;
    elseif (filled == 5)
        isOk = 0;

        % Corner case top left
        partial_filled = 0;
        for i = -2:0
            for j = -2:0
                if (check_boundaries(pixel_i + i, pixel_j + j) == 1 && ...
                    img_canny(pixel_i + i, pixel_j + j) ~= 0)
                    partial_filled = partial_filled + 1;
                end
            end
        end
        if (partial_filled == 5) 
            isOk = 1;
        end

         % Corner case top right
        partial_filled = 0;
        for i = -2:0
            for j = 0:2
                if (check_boundaries(pixel_i + i, pixel_j + j) == 1 && ...
                    img_canny(pixel_i + i, pixel_j + j) ~= 0)
                    partial_filled = partial_filled + 1;
                end
            end
        end
        if (partial_filled == 5) 
            isOk = 1;
        end

         % Corner case bottom left
        partial_filled = 0;
        for i = 0:2
            for j = -2:0
                if (check_boundaries(pixel_i + i, pixel_j + j) == 1 && ...
                    img_canny(pixel_i + i, pixel_j + j) ~= 0)
                    partial_filled = partial_filled + 1;
                end
            end
        end
        if (partial_filled == 5) 
            isOk = 1;
        end

         % Corner case bottom right
        partial_filled = 0;
        for i = 0:2
            for j = 0:2
                if (check_boundaries(pixel_i + i, pixel_j + j) == 1 && ...
                    img_canny(pixel_i + i, pixel_j + j) ~= 0)
                    partial_filled = partial_filled + 1;
                end
            end
        end
        if (partial_filled == 5) 
            isOk = 1;
        end

        % Corner case perfect L 
        if (check_boundaries(pixel_i - 1, pixel_j) == 1 && ...
            check_boundaries(pixel_i, pixel_j - 1) == 1 && ...
            img_canny(pixel_i - 1, pixel_j) ~= 0 && ...
            img_canny(pixel_i, pixel_j - 1) ~= 0)
            isOk = 0;
        end 

        % Corner case perfect L 
        if (check_boundaries(pixel_i - 1, pixel_j) == 1 && ...
            check_boundaries(pixel_i, pixel_j + 1) == 1 && ...
            img_canny(pixel_i - 1, pixel_j) ~= 0 && ...
            img_canny(pixel_i, pixel_j + 1) ~= 0)
            isOk = 0;
        end 

        % Corner case perfect L 
        if (check_boundaries(pixel_i + 1, pixel_j) == 1 && ...
            check_boundaries(pixel_i, pixel_j - 1) == 1 && ...
            img_canny(pixel_i + 1, pixel_j) ~= 0 && ...
            img_canny(pixel_i, pixel_j - 1) ~= 0)
            isOk = 0;
        end 

        % Corner case perfect L 
        if (check_boundaries(pixel_i + 1, pixel_j) == 1 && ...
            check_boundaries(pixel_i, pixel_j + 1) == 1 && ...
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