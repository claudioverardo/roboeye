function [qloc, fval, info]=inverse_kin_simple(transl,eulr,startingpos_in)
%COMPUTE INVERSE KINEMATICS

  startingpos=startingpos_in([2 4]);

  %set fsolve options
  options = optimoptions('fsolve','MaxIterations',50000,'MaxFunctionEvaluations',50000,'Display','off'); 
  %define Roto-translation matrix in the end effector's rf
  
  %solve numerically inverse kinematics problem finding the nearest zero of inv_kin_prob from startingpos
  iks_obj = @(x)inv_kin_prob_simple(x,transl,eulr(2));
  [var, fval, info] = fsolve (iks_obj, startingpos,options);
  
  qloc_2(1)=var(1);
  qloc_2(2)=var(2)-var(1);
  
  %sin_var
  
  %qloc_2(1)=asin(sin_var(1))*180/pi;
  %qloc_2(2)=asin(sin_var(2))*180/pi;
  
%   qloc(1)=eulr(1)*180/pi;
%   qloc(2)=-qloc_2(1);
%   qloc(3)=eulr(2)+qloc_2(1)+qloc_2(2);
%   qloc(4)=-qloc_2(2);
%   qloc(5)=eulr(3)*180/pi;

   qloc(1)=atan2(transl(2),abs(transl(1)))*180/pi;
   if transl(1)<0
       qloc(2)=qloc_2(1)*180/pi;
       qloc(3)=qloc_2(2)*180/pi;
       qloc(4)=(eulr(2)-var(2))*180/pi;       
   else
       qloc(2)=-qloc_2(1)*180/pi;
       qloc(3)=-qloc_2(2)*180/pi;
       qloc(4)=-(eulr(2)-var(2))*180/pi;
   end
   qloc(5)=eulr(3)*180/pi;
      
end

function sol=inv_kin_prob_simple(var, transl, eulr2)
%define the equation which has to be numerically solved in order to comput inverse kinematics
    
  braccio=[71 125 125 195 0];
  
  x=transl(1);
  y=transl(2);
  z=transl(3);
  
  cos1=sqrt(1-var(1)^2);
  cos2=sqrt(1-var(2)^2);

    
  r=sqrt(x^2+y^2);
  
  %sol(1)=braccio(2)*var(1)+braccio(3)*(sin(-eulr2)*cos2+cos(-eulr2)*(-var(2)))+braccio(4)*sin(-eulr2)-r;
  %sol(2)=braccio(2)*cos1+braccio(3)*(cos(-eulr2)*cos2-sin(-eulr2)*(-var(2)))+braccio(4)*cos(-eulr2)+braccio(1)-z;
  
  sol(1)=braccio(2)*sin(var(1))+braccio(3)*sin(var(2))+braccio(4)*sin(eulr2)-r;
  sol(2)=braccio(2)*cos(var(1))+braccio(3)*cos(var(2))+braccio(4)*cos(eulr2)-z+braccio(1);
  
end
