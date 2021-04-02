%define the equation which has to be numerically solved in order to comput inverse kinematics

function sol=inv_kin_prob_super_simple(var)
  
  %variable definition
  %var(1)=q2
  %var(2)=q2+q3
  
  global transl2;
  global q4rad;
    
  braccio=[71 125 125 195 0];
  
  x=transl2(1);
  y=transl2(2);
  z=transl2(3);
  
%   cos1=sqrt(1-var(1)^2);
%   cos2=sqrt(1-var(2)^2);

    
  r=sqrt(x^2+y^2);
  
  %sol(1)=braccio(2)*var(1)+braccio(3)*(sin(-eulr2)*cos2+cos(-eulr2)*(-var(2)))+braccio(4)*sin(-eulr2)-r;
  %sol(2)=braccio(2)*cos1+braccio(3)*(cos(-eulr2)*cos2-sin(-eulr2)*(-var(2)))+braccio(4)*cos(-eulr2)+braccio(1)-z;
  
%   sol(1)=braccio(2)*sin(var(1))+braccio(3)*sin(var(2))+braccio(4)*sin(eulr2)-r;
%   sol(2)=braccio(2)*cos(var(1))+braccio(3)*cos(var(2))+braccio(4)*cos(eulr2)-z+braccio(1);

%  sol(1)=braccio(2)*sin(var(1))+(braccio(3)+braccio(4)*cos(q4rad))*sin(var(2))+braccio(4)*sin(q4rad)*cos(var(2))-r;
%  sol(2)=braccio(2)*cos(var(1))+(braccio(3)+braccio(4)*cos(q4rad))*cos(var(2))-braccio(4)*sin(q4rad)*sin(var(2))-z+braccio(1);
  
  sol(1)=braccio(2)*sin(var(1))+braccio(3)*sin(var(2))+braccio(4)*sin(var(2)+q4rad)-r;
  sol(2)=braccio(2)*cos(var(1))+braccio(3)*cos(var(2))+braccio(4)*cos(var(2)+q4rad)-z+braccio(1);
end
