function [qloc, fval, info]=inverse_kin(transl,eulr,startingpos_in)
%COMPUTE INVERSE KINEMATICS

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
  iks_obj = @(x) inv_kin_prob2(x);
  [qloc_3, fval, info] = fsolve (iks_obj, startingpos,options);
  
  %qloc=qloc_3;
  qloc=[eulr(1)*180/pi qloc_3 eulr(3)*180/pi];
  
  %fval is the actual function's value at the found solution
  
  %info gives information about the solution (i.e. if it converged or not)
  
end


function sol=inv_kin_prob(q)
%define the equation which has to be numerically solved in order to comput inverse kinematics
  
  global A_target;


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
  
  Asol=(direct_kin(q,5)-A_target)./norm;  
 
 
  for i=1:5
     sol(i)=Asol(chosen_index(i,1),chosen_index(i,2));
  end
  
  %sol=[Asol([1 2 3],4)' Asol(inndex(1,1),inndex(1,1)) Asol(inndex(2,1),inndex(2,1))];
  sol=Asol;

  
end


function sol=inv_kin_prob2(q_in)
%define the equation which has to be numerically solved in order to comput inverse kinematics
  
  global A_target;
  global q_set;


  %position in the matrix equation in which extract the system
  
  
  %%%%%%% convert model's angle convention to Devenit-Hartenberg convention
  q=[q_set(1) q_in q_set(2)];
  
  neutral_position=[0 90 0 -90 0]; 
  
  q=q+neutral_position;
  
  Asol=direct_kin(q,5)-A_target;  
  
  %sol=[Asol([1 2 3],4)' Asol(inndex(1,1),inndex(1,1)) Asol(inndex(2,1),inndex(2,1))];
  sol=Asol;

  
end


function sol=inv_kin_prob3(q_in)
%define the equation which has to be numerically solved in order to comput inverse kinematics
  
  global A_target;
  global q_set;


  %position in the matrix equation in which extract the system
  
  chosen_index=[1 4;
               3 4;
               2 2;
               3 3];
  
  
  %%%%%%% convert model's angle convention to Devenit-Hartenberg convention
  q=[q_set(1) q_in];
  
  neutral_position=[0 90 0 -90 0]; 
  
  q=q+neutral_position;
  
  Asol=direct_kin(q,5)-A_target;  
 
 
  for i=1:length(chosen_index(:,1))
     sol(i)=Asol(chosen_index(i,1),chosen_index(i,2));
  end
  
  %sol=[Asol([1 2 3],4)' Asol(inndex(1,1),inndex(1,1)) Asol(inndex(2,1),inndex(2,1))];
  sol=Asol;

  
end

