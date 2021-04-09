function R = rot_mat(alpha)
% ROT_MAT Compute a rotation matrix in 2D space.
%
%   R = ROT_MAT(alpha)
%
%   Input arguments:
%   ------------------
%   alpha:  rotation angle
%
%   Output arguments:
%   ------------------
%   R:      2x2 rotation matrix
%
% See also ROT3D_MAT

    R=[cos(alpha) -sin(alpha);
       sin(alpha) cos(alpha)];  
   
end
