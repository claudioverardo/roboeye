function Rt=roto_transl_mat(transl,eulr)
  
  phi=eulr(1);
  theta=eulr(2);
  psi=eulr(3);
  Rt=eye(4);
  Rt([1 2 3],[1,2,3])=rot3d_mat(phi,3)*rot3d_mat(theta,1)*rot3d_mat(psi,3);
  Rt([1 2 3],4)=transl;
end
