%COMPUTE INVERSE KINEMATICS

function [qloc, fval, info]=inverse_kin(transl,eulr,startingpos)
  
  %define Roto-translation matrix in the end effector's rf
  global A_target
  
  A_target=roto_transl_mat(transl,eulr);
  
  %solve numerically inverse kinematics problem finding the nearest zero of inv_kin_prob from startingpos
  [qloc, fval, info] = fsolve ("inv_kin_prob", startingpos);
  
  %fval is the actual function's value at the found solution
  
  %info gives information about the solution (i.e. if it converged or not)
  
endfunction
