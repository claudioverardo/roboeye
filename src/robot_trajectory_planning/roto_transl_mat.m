function Rt = roto_transl_mat(transl, eulr)
% ROTO_TRANSL_MAT Compute a rototranslation matrix in 3D space.
%
%   Rt = ROTO_TRANSL_MAT(transl, eulr)
%
%   Input arguments:
%   ------------------
%   transl: translation vector
%   eulr:   [phi theta psi] aka ZYZ parametrization of rotation,
%           cf. rot3d_mat(...)
%
%   Output arguments:
%   ------------------
%   Rt:     4x4 rototranslation matrix, Rt = [R(eulr) transl; 0 1]
%
% See also ROT3D_MAT
  
    Rt=eye(4);
    Rt(1:3,1:3)=rot3d_mat(eulr);
    Rt(1:3,4)=transl;
  
end
