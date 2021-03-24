%define the equation which has to be numerically solved in order to comput inverse kinematics

function sol=inv_kin_prob3(q_in)
  
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
