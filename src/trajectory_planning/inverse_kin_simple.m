%COMPUTE INVERSE KINEMATICS

function [qloc, fval, info]=inverse_kin_simple(transl,eulr,startingpos_in)

  startingpos=startingpos_in([2 4]);

  %set fsolve options
  options = optimoptions('fsolve','MaxIterations',50000,'MaxFunctionEvaluations',50000,'Display','off'); 
  %define Roto-translation matrix in the end effector's rf
  global transl2
  global eulr2
  
  transl2=transl;
  eulr2=eulr(2);
  
  %solve numerically inverse kinematics problem finding the nearest zero of inv_kin_prob from startingpos
  [var, fval, info] = fsolve ("inv_kin_prob_simple", startingpos,options);
  
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
