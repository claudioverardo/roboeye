function check_ans = check_boundaries(i, j, img_size)
% CHECK_BOUNDARIES  Match Aruco markers with candidate ROIs.
%
%   check_ans = CHECK_BOUNDARIES(i, j, img_size) the function returns 1 if
%   the coordinates of the point (i, j) are inside the img_size. 
%
%   Input arguments:
%   ------------------
%   i:          i point coordinate (row)
%   j:          j point coordinate (column)
%   img_size:   1x2 (rows img limit, columns img limit)
%
%   Output arguments:
%   ------------------
%   check_ans:  1 if the point is inside the image size 0 otherwiae
%
%   See also ROI_EXTRACTION_DFS


    check_ans = 0;
    
    if (i >= 1 && i <= img_size(1)) && (j >= 1 && j <= img_size(2))
       check_ans = 1;
    end
    
end