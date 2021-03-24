%define the equation which has to be numerically solved in order to comput inverse kinematics

function sol=inv_kin_prob2(q_in)
  
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
