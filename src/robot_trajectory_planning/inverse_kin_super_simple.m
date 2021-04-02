%COMPUTE INVERSE KINEMATICS

function [qloc, fval, info]=inverse_kin_super_simple(transl,joint4,startingpos_in)

  startingpos=startingpos_in([2 4]);

  %set fsolve options
  options = optimoptions('fsolve','MaxIterations',50000,'MaxFunctionEvaluations',50000,'Display','off'); 
  %define Roto-translation matrix in the end effector's rf
  global transl2
  global q4rad
  
  transl2=transl;
  if transl(1)<0 
     q4rad=joint4*pi/180;
  else
     q4rad=-joint4*pi/180;
  end

  %solve numerically inverse kinematics problem finding the nearest zero of inv_kin_prob from startingpos
  [var, fval, info] = fsolve ("inv_kin_prob_super_simple", startingpos,options);
  
  qloc_2(1)=var(1);
  qloc_2(2)=var(2)-var(1);
  
   qloc(1)=atan2(transl(2),abs(transl(1)))*180/pi;
   if transl(1)<0
       qloc(2)=qloc_2(1)*180/pi;
       qloc(3)=qloc_2(2)*180/pi;
       qloc(4)=joint4;       
   else
       qloc(2)=-qloc_2(1)*180/pi;
       qloc(3)=-qloc_2(2)*180/pi;
       qloc(4)=joint4;
   end
   qloc(5)=0; %for the moment
      
end
