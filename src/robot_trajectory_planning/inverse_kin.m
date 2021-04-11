function [qloc, fval, info] = inverse_kin(transl, eulr, startingpos_in, braccio_params)
% INVERSE_KIN Solve the general problem of inverse kinematics for a given 
% position and orientation of the end effector.
%
%   [qloc, fval, info] = INVERSE_KIN(transl, eulr, startingpos_in, braccio_params)
%
%   Input arguments:
%   ------------------
%   transl:             translation vector of the end effector
%   eulr:               euler angles of the rotation of the end effector
%   startingpos_in:     1xQNUM-1 array, initial guess for the solver
%   braccio_params:     1xQNUM-1 array, real parameters of the Braccio robot,
%                       cf. direct_kin(...)
%
%   Output arguments:
%   ------------------
%   qloc:               1xQNUM-1 array, solution found (model convention)
%   fval:               final residual of the solver
%   info:               final flag of the solver
%
%   NOTE: this method is rather unstable, cf. inverse_kin_super_simple(...)
%         for a more stable solution.
%
% See also INVERSE_KIN_SUPER_SIMPLE
  
  %define Roto-translation matrix in the end effector's rf
  A_target=roto_transl_mat(transl,eulr);
  
  %set fsolve options
  options = optimoptions('fsolve','MaxIterations',50000,'MaxFunctionEvaluations',50000,'Display','iter'); 

  %solve numerically inverse kinematics problem finding the nearest zero of inv_kin_prob from startingpos
  
  %search in all the joints space
  startingpos=startingpos_in;
  iks_obj1 = @(x) inv_kin_prob1(x,A_target,braccio_params);
  [qloc, fval, info] = fsolve (iks_obj1,startingpos,options);
  
  %search only in the space of joints 2-3-4
  %q_set=[eulr(1) eulr(3)]*180/pi;
  %startingpos=startingpos_in([2 3 4]);
  %iks_obj2 = @(x) inv_kin_prob2(x,A_target,q_set,braccio_params);
  %iks_obj3 = @(x) inv_kin_prob3(x,A_target,q_set,braccio_params);
  %[qloc_3, fval, info] = fsolve (iks_obj2,startingpos,options);
  %qloc=[eulr(1)*180/pi qloc_3 eulr(3)*180/pi];
  
  %fval is the actual function's value at the found solution
  %info gives information about the solution (i.e. if it converged or not)
  
end


function sol=inv_kin_prob1(q, A_target, braccio_params)
%define the equation which has to be numerically solved in order to comput inverse kinematics


  %position in the matrix equation in which extract the system
  
  chosen_index=[1 4;
               2 4;
               3 4;
               1 1;
               3 3];
  
  
  %%%%%%% convert model's angle convention to Devenit-Hartenberg convention
  
  neutral_position=[0 90 0 -90 0]; 
  
  q=q+neutral_position;
  
  norm=ones(4);
  norm(4,[1 2 3])=[300 300 300];
  
  Asol=(direct_kin(q,5,braccio_params)-A_target)./norm;  
 
 
  for i=1:5
     sol(i)=Asol(chosen_index(i,1),chosen_index(i,2));
  end
  
  %sol=[Asol([1 2 3],4)' Asol(inndex(1,1),inndex(1,1)) Asol(inndex(2,1),inndex(2,1))];
  sol=Asol;

  
end


function sol=inv_kin_prob2(q_in, A_target, q_set, braccio_params)
%define the equation which has to be numerically solved in order to comput inverse kinematics


  %position in the matrix equation in which extract the system
  
  
  %%%%%%% convert model's angle convention to Devenit-Hartenberg convention
  q=[q_set(1) q_in q_set(2)];
  
  neutral_position=[0 90 0 -90 0]; 
  
  q=q+neutral_position;
  
  Asol=direct_kin(q,5,braccio_params)-A_target;  
  
  %sol=[Asol([1 2 3],4)' Asol(inndex(1,1),inndex(1,1)) Asol(inndex(2,1),inndex(2,1))];
  sol=Asol;

  
end


function sol=inv_kin_prob3(q_in, A_target, q_set, braccio_params)
%define the equation which has to be numerically solved in order to comput inverse kinematics

  %position in the matrix equation in which extract the system
  
  chosen_index=[1 4;
               3 4;
               2 2;
               3 3];
  
  
  %%%%%%% convert model's angle convention to Devenit-Hartenberg convention
  q=[q_set(1) q_in];
  
  neutral_position=[0 90 0 -90 0]; 
  
  q=q+neutral_position;
  
  Asol=direct_kin(q,5,braccio_params)-A_target;  
 
 
  for i=1:length(chosen_index(:,1))
     sol(i)=Asol(chosen_index(i,1),chosen_index(i,2));
  end
  
  %sol=[Asol([1 2 3],4)' Asol(inndex(1,1),inndex(1,1)) Asol(inndex(2,1),inndex(2,1))];
  sol=Asol;

  
end

