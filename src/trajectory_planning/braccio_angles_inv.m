% convert angles from model convention to robot conv 
function in=braccio_angles_inv(out)

  out(3)=out(3)+5;
  out(4)=out(4)-4;
  out(2)=out(2)+1;
  
  AHposition=[90 84 99 95 90];%Actual home position
  AHposition=[90 84 99 95 90];%%Actual home position downgrade 25/03/21
  
  
  THposition=[90 90 90 90 90];%Theoretical home position
  
  incr=AHposition-THposition;

  in=out-[90 90 90 90 90]-incr;
  
  in(1)=-in(1);
  in(4)=-in(4);

end