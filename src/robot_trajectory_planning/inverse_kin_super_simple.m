function [qloc, fval, info] = inverse_kin_super_simple(transl, joint4, startingpos_in, braccio_params)
% INVERSE_KIN_SUPER_SIMPLE Solve the problem of inverse kinematics for a given 
% position of the end effector. Differently to the function inverse_kin(...)
% it calculates the 1st and the 5th joints positions via geometric considerations.
% Differently to the function inverse_kin_simple(...), it receives as input the 
% target position of the 4th joint. Then, it solves a super-simplified version
% of the inverse kinematics problem on the remaining 2 joints (2-3).
%
%   [qloc, fval, info] = INVERSE_KIN_SUPER_SIMPLE(transl, joint4, startingpos_in,
%   braccio_params)
%
%   Input arguments:
%   ------------------
%   transl:             translation vector of the end effector
%   joint4:             angular position of the 4th joint 
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
% See also GOTHERE

  startingpos=startingpos_in([2 4]);

  %set fsolve options
  options = optimoptions('fsolve','MaxIterations',50000,'MaxFunctionEvaluations',50000,'Display','off'); 
  %define Roto-translation matrix in the end effector's rf

  if transl(1)<0 
     q4rad=joint4*pi/180;
  else
     q4rad=-joint4*pi/180;
  end

  %solve numerically inverse kinematics problem finding the nearest zero of inv_kin_prob from startingpos
  iks_obj = @(x) inv_kin_prob_super_simple(x,transl,q4rad,braccio_params);
  
  [var, fval, info] = fsolve (iks_obj, startingpos,options);
  
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

function sol=inv_kin_prob_super_simple(var,transl,q4rad,braccio_params)
%define the equation which has to be numerically solved in order to comput inverse kinematics
  
  %variable definition
  %var(1)=q2
  %var(2)=q2+q3
    
  %braccio_params=[71 125 125 195 0];
  
  x=transl(1);
  y=transl(2);
  z=transl(3);
  
  %Adjust last parameter for negative semi-workspace
  if x>=0
      delta = -braccio_params(5);
  else
      delta = braccio_params(5);
  end
  
%   cos1=sqrt(1-var(1)^2);
%   cos2=sqrt(1-var(2)^2);

    
  r=sqrt(x^2+y^2);
  
  %sol(1)=braccio(2)*var(1)+braccio(3)*(sin(-eulr2)*cos2+cos(-eulr2)*(-var(2)))+braccio(4)*sin(-eulr2)-r;
  %sol(2)=braccio(2)*cos1+braccio(3)*(cos(-eulr2)*cos2-sin(-eulr2)*(-var(2)))+braccio(4)*cos(-eulr2)+braccio(1)-z;
  
%   sol(1)=braccio(2)*sin(var(1))+braccio(3)*sin(var(2))+braccio(4)*sin(eulr2)-r;
%   sol(2)=braccio(2)*cos(var(1))+braccio(3)*cos(var(2))+braccio(4)*cos(eulr2)-z+braccio(1);

%  sol(1)=braccio(2)*sin(var(1))+(braccio(3)+braccio(4)*cos(q4rad))*sin(var(2))+braccio(4)*sin(q4rad)*cos(var(2))-r;
%  sol(2)=braccio(2)*cos(var(1))+(braccio(3)+braccio(4)*cos(q4rad))*cos(var(2))-braccio(4)*sin(q4rad)*sin(var(2))-z+braccio(1);
  
  %%%%%%%   WITHOUT DELTA  %%%%%%%%
  sol(1)=braccio_params(2)*sin(var(1))+braccio_params(3)*sin(var(2))+braccio_params(4)*sin(var(2)+q4rad)-r;
  sol(2)=braccio_params(2)*cos(var(1))+braccio_params(3)*cos(var(2))+braccio_params(4)*cos(var(2)+q4rad)-z+braccio_params(1);
  
  %%%%%%%   WITH DELTA  %%%%%%%%
  %sol(1)=braccio_params(2)*sin(var(1))+braccio_params(3)*sin(var(2))+braccio_params(4)*sin(var(2)+q4rad)-r+delta*sin(var(2)+q4rad-pi/2);
  %sol(2)=braccio_params(2)*cos(var(1))+braccio_params(3)*cos(var(2))+braccio_params(4)*cos(var(2)+q4rad)-z+braccio_params(1)+delta*cos(var(2)+q4rad-pi/2);
  
end
