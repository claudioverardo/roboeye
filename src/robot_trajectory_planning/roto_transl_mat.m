function Rt = roto_transl_mat(transl, eulr)
% ROTO_TRANSL_MAT Compute a rototranslation matrix in 3D space.
%
%   Rt = ROTO_TRANSL_MAT(transl, eulr)
%
%   Input arguments:
%   ------------------
%   transl: 3x1 vector, translation vector
%   eulr:   3x1 vector, [phi,theta,psi] parametrization of rotation
%
%   Output arguments:
%   ------------------
%   Rt:     4x4 rototranslation matrix
%
% See also ROT3D_MAT
  
    phi=eulr(1);
    theta=eulr(2);
    psi=eulr(3);
    Rt=eye(4);
    Rt([1 2 3],[1,2,3])=rot3d_mat(psi,3)*rot3d_mat(theta,2)*rot3d_mat(phi,3);
    Rt([1 2 3],4)=transl;
  
end
