function [qloc, errorflag] = inverse_kin_super_simple_an(transl, q4, braccio_params)
% INVERSE_KIN_SUPER_SIMPLE_AN Solve the problem of inverse kinematics for a given 
% position of the end effector. Differently to the function inverse_kin(...)
% it calculates the 1st and the 5th joints positions via geometric considerations.
% Differently to the function inverse_kin_simple(...), it receives as input the 
% target position of the 4th joint. Then, it solves analytically a super-simplified
% version of the inverse kinematics problem on the remaining 2 joints (2-3).
%
%   [qloc, errorflag] = INVERSE_KIN_SUPER_SIMPLE_AN(transl, q4, braccio_params)
%
%   Input arguments:
%   ------------------
%   transl:             translation vector of the end effector
%   q4:                 angular position of the 4th joint 
%   braccio_params:     1xQNUM-1 array, real parameters of the Braccio robot,
%                       cf. direct_kin(...)
%
%   Output arguments:
%   ------------------
%   qloc:               1xQNUM-1 array, solution found (model convention)
%   errorflag:          1 if solution is found, -1 otherwise
%
% See also GENERATE_TRAJECTORY, GOTHERE

  %--- q1 and q5 computation
  
    errorflag = 1;
  
    % joint 1
    qloc(1) = atan2(transl(2),abs(transl(1)))*180/pi;
    
    % joint 5
    qloc(5)=0; % for the moment
  
  %--- PLANAR ROBOT SOLUTION 
  
  r = sign(transl(1))*sqrt(transl(1)^2+transl(2)^2);
  
  % convert parameters using siciliano's convention
    
    q4 = q4*pi/180;
    a1 = braccio_params(2);
    a2 = sqrt(braccio_params(3)^2+braccio_params(4)^2+...
         2*braccio_params(3)*braccio_params(4)*cos(q4));
    
    % axes are rotated
    pwx = transl(3)-braccio_params(1);
    pwy = -r;
    
    % main computation
    
    c2 = (pwx^2+pwy^2-a1^2-a2^2)/(2*a1*a2);
    if (1-c2^2)<0
        errorflag = -1;
        qloc = [0 0 0 0 0];
    else
        s2 = -sqrt(1-c2^2);

        theta2 = atan2(s2,c2);

        s1 = ((a1+a2*c2)*pwy-a2*s2*pwx)/(pwx^2+pwy^2);
        c1 = ((a1+a2*c2)*pwx+a2*s2*pwy)/(pwx^2+pwy^2);

        if abs(s1)>1 || abs(c1)>1
            errorflag = -1;
            qloc = [0 0 0 0 0];
        else

            theta1 = atan2(s1,c1);

            c1_5 = (braccio_params(3)+braccio_params(4)*cos(q4))/a2;

            theta1_5 = abs(acos(c1_5))*sign(q4);

            % jointS 2,3,4
            qloc(2) = theta1*180/pi;
            qloc(3) = (theta2-theta1_5)*180/pi;
            qloc(4) = q4*180/pi;
        end
    end
end
