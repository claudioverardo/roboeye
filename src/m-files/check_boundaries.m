function check_ans = check_boundaries(i, j, img_size)
% check if the ij coordinates of a point are valid

    check_ans = 0;
    
    if (i >= 1 && i <= img_size(1)) && (j >= 1 && j <= img_size(2))
       check_ans = 1;
    end
    
end