function R = rot3d_mat(alpha, dir)
% ROT3D_MAT Compute a rotation matrix in 3D space around x, y or z.
%
%   R = ROT3D_MAT(alpha, dir)
%
%   Input arguments:
%   ------------------
%   alpha:  rotation angle
%   dir:    rotation direction (1 = x axis, 2 = y axis, 3 = z axis)
%
%   Output arguments:
%   ------------------
%   R:      3x3 rotation matrix
%
% See also ROT_MAT

    if dir==1
        ipos=[2 3];
    elseif dir==2
        ipos=[1 3];
        alpha=-alpha;%change sign for y;
    else
        ipos=[1 2];
    end
    R=eye(3);
    R([ipos],[ipos])=rot_mat(alpha);

end
