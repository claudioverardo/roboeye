function Atot = direct_kin(q, njoints, braccio_params)
% DIRECT_KIN Compute the direct kinematics of the Braccio robot for a given
% joints position and a set of real parameters of the robot. With njoints=QNUM-1
% the direct kinematics for all the joints is computed. With njoints<QNUM-1
% the direct kinematics of only the first njoints is computed.
% 
%   Atot = DIRECT_KIN(q, njoints, braccio_params)
%
%   Input arguments:
%   ------------------
%   q:              1xQNUM-1 array, joints position in model convention
%   njoints:        number of joints to be considered for direct kinematics
%   braccio_params: 1xQNUM-1 array, real parameters of the Braccio robot
%                   - (1) = distance between ground and joint 1
%                   - (2) = distance between joint 1 and joint 2
%                   - (3) = distance between joint 2 and joint 3
%                   - (4) = distance between joint 3 and EF tip
%                   - (5) = 'a' (aka 'r') DH parameter for joint 5
%
%   Output arguments:
%   ------------------
%   Atot:           4x4 rototranslation matrix of direct kinematics
%
% See also DENAVIT_HARTERNBERG
    
  q=q*pi/180; %convert to rad

  % if nargin <= 2
  %     % braccio_params=[67 125 125 188 0]; %lengths of Braccio's segments %Since 19_03_21
  %     braccio_params=[71 125 125 195 0];
  % end
  
  %delta=-4.5;
  delta=braccio_params(5);

  %DH parameters
  a=[0 braccio_params(2:3) 0 delta]; 
  d=[braccio_params(1) 0 0 0 braccio_params(4)];
  alpha=[90 0 0 -90 0]*pi/180; % MOD 1 in rad

  %alpha=[90 0 0 90 0]*pi/180; % MOD 2 in rad

  %%%%% PREVIOUS TESTS WITH CAD %%%%%%
  %q=[0 27.96 97.37 54.66 0]*pi/180;
  %q=[0 27.96 97.37 54.66+90 0]*pi/180;
  %q=[0 27.96 97.37 -35.33 0]*pi/180; % MOD1 from "intuitive" counter-clockwise rotation
  
  Atot=eye(4);
  
  if njoints>length(d)
      njoints=length(d);
  end
  
  if njoints>0
      for i=1:njoints
        Atot=Atot*denavit_harternberg(q(i),d(i),alpha(i),a(i));
      end
  end
  
end
