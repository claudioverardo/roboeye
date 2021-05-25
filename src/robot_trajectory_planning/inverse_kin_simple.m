function [qloc, fval, info] = inverse_kin_simple(transl, eulr, startingpos_in, braccio_params)
% INVERSE_KIN_SIMPLE Solve the problem of inverse kinematics for a given 
% position and orientation of the end effector. Differently to the function
% inverse_kin(...) it calculates the 1st and the 5th joints positions via 
% geometric considerations. Then, it solves numerically a simplified version 
% of the inverse kinematics problem on the remaining 3 joints (2-3-4).
%
%   [qloc, fval, info] = INVERSE_KIN_SIMPLE(transl, eulr, startingpos_in, 
%   braccio_params)
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
% See also GENERATE_TRAJECTORY, TOUCHDOWN

  startingpos=startingpos_in([2 4]);

  %set fsolve options
  options = optimoptions('fsolve','MaxIterations',50000,'MaxFunctionEvaluations',50000,'Display','off'); 
  %define Roto-translation matrix in the end effector's rf
  
  %solve numerically inverse kinematics problem finding the nearest zero of inv_kin_prob from startingpos
  iks_obj = @(x)inv_kin_prob_simple(x,transl,eulr(2),braccio_params);
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

function sol=inv_kin_prob_simple(var, transl, eulr2, braccio_params)
%define the equation which has to be numerically solved in order to comput inverse kinematics
    
  % braccio_params=[71 125 125 195 0];
  
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
  
  %%%%%%%   WITHOUT DELTA  %%%%%%%%
  sol(1)=braccio_params(2)*sin(var(1))+braccio_params(3)*sin(var(2))+braccio_params(4)*sin(eulr2)-r;
  sol(2)=braccio_params(2)*cos(var(1))+braccio_params(3)*cos(var(2))+braccio_params(4)*cos(eulr2)-z+braccio_params(1);

  %%%%%%%   WITH DELTA  %%%%%%%%
  %sol(1)=braccio_params(2)*sin(var(1))+braccio_params(3)*sin(var(2))+braccio_params(4)*sin(eulr2)-r+delta*sin(eulr2-pi/2);
  %sol(2)=braccio_params(2)*cos(var(1))+braccio_params(3)*cos(var(2))+braccio_params(4)*cos(eulr2)-z+braccio_params(1)+delta*cos(eulr2-pi/2);
  
end
