function Atot= direct_kin(q,njoints)
  %COMPUTE DIRECT KINEMATICS OF BRACCIO
  %can also campute direct kinematics of the first "njoints" joints with njoints<=5
    
  q=q*pi/180; %convert to rad

  braccio=[67 125 125 188 0]; %lengths of Braccio's segments
  
  delta=-4.5;

  %DH parameters
  a=[0 braccio(2:3) 0 delta]; 
  d=[braccio(1) 0 0 0 braccio(4)+braccio(5)];
  alpha=[90 0 0 -90 0]*pi/180; % MOD 1 in rad

  %alpha=[90 0 0 90 0]*pi/180; % MOD 2 in rad

  %%%%% PREVIOUS TESTS WITH CAD %%%%%%
  %q=[0 27.96 97.37 54.66 0]*pi/180;
  %q=[0 27.96 97.37 54.66+90 0]*pi/180;
  %q=[0 27.96 97.37 -35.33 0]*pi/180; % MOD1 from "intuitive" counter-clockwise rotation

  Atot=eye(4);

  for i=1:njoints
    Atot=Atot*denavit_harternberg(q(i),d(i),alpha(i),a(i));
  end
end
