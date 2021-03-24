% convert angles from model convention to robot conv 
function out=braccio_angles(in)
  
  AHposition=[90 84 99 95 90];%Actual home position
  AHposition=[90 84 99 91 90];%Actual home position update 24/03/21
  AHposition=[90 83 98 97 90];%Actual home position update 24/03/21
  
  THposition=[90 90 90 90 90];%Theoretical home position
  
  incr=AHposition-THposition;
  
  in(1)=-in(1);
  in(4)=-in(4);

  out=in+[90 90 90 90 90]+incr;


end