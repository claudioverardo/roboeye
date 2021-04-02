%COMPUTE INVERSE KINEMATICS

function [qloc, fval, info]=inverse_kin(transl,eulr,startingpos_in)

  startingpos=startingpos_in([2 3 4]);
  %startingpos=startingpos_in;

  %set fsolve options
  options = optimoptions('fsolve','MaxIterations',50000,'MaxFunctionEvaluations',50000); 
  %define Roto-translation matrix in the end effector's rf
  global A_target
  global q_set
  
  A_target=roto_transl_mat(transl,eulr);
  q_set=[eulr(1) eulr(3)]*180/pi;
  
  %solve numerically inverse kinematics problem finding the nearest zero of inv_kin_prob from startingpos
  [qloc_3, fval, info] = fsolve ("inv_kin_prob2", startingpos,options);
  
  %qloc=qloc_3;
  qloc=[eulr(1)*180/pi qloc_3 eulr(3)*180/pi];
  
  %fval is the actual function's value at the found solution
  
  %info gives information about the solution (i.e. if it converged or not)
  
end
