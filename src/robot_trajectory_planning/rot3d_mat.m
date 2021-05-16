function R = rot3d_mat(eulr)
% ROT3D_MAT Compute a rotation matrix in 3D space.
%
%   R = ROT3D_MAT(eulr)
%
%   Input arguments:
%   ------------------
%   eulr:   [phi theta psi] aka ZYZ parametrization of the rotation
%           - eulr(1) rotation angle around z-axis
%           - eulr(2) rotation angle around y-axis
%           - eulr(3) rotation angle around z-axis
%
%   Output arguments:
%   ------------------
%   R:      3x3 rotation matrix, R = Rz(psi)*Ry(theta)*Rz(phi)
%
% See also ROT_MAT

    phi=eulr(1);
    theta=eulr(2);
    psi=eulr(3);
    R=rot3d_mat_dir(psi,3)*rot3d_mat_dir(theta,2)*rot3d_mat_dir(phi,3);

end

function R = rot3d_mat_dir(alpha, dir)
% rotations around x,y,z axes (dir=1,2,3 respectively)

    if dir==1
        ipos=[2 3];
    elseif dir==2
        ipos=[1 3];
        alpha=-alpha;%change sign for y;
    else
        ipos=[1 2];
    end
    R=eye(3);
    R(ipos,ipos)=rot_mat(alpha);

end
