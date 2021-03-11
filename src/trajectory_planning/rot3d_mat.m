function R=rot3d_mat(alpha,dir)
  if dir==1
    ipos=[2 3];
  elseif dir==2
    ipos=[1 3];
  else
    ipos=[1 2];
  end
  R=eye(3);
  R([ipos],[ipos])=rot_mat(alpha);
end
