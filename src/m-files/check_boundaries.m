function ans = check_boundaries(i, j)
    global img_canny;
    
    if (i >= 1 && i <= size(img_canny, 1)) && ...
       (j >= 1 && j <= size(img_canny, 2))
       ans = 1;
       return;
    end
    
    ans = 0;
end