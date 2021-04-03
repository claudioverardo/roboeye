% convert angles from model convention to robot conv 
function out=braccio_angles(in,post_corr,AHposition,THposition)
  
  if nargin <=2
      
%   AHposition=[90 84 99 95 90];%Actual home position
%   AHposition=[90 84 99 91 90];%Actual home position update 24/03/21
%   AHposition=[90 83 98 97 90];%Actual home position update 24/03/21
  
    AHposition=[90 84 99 95 90];%%Actual home position downgrade 25/03/21
  end
  
  if nargin <=3
    THposition=[90 90 90 90 90];%Theoretical home position
  end
  
  incr=AHposition-THposition;
  
  in(1)=-in(1);
  in(4)=-in(4);

  out=in+[90 90 90 90 90]+incr;
  
  % a posteriori experimental corrections
  
%   %out(3)=out(3)-5;
%   out(3)=out(3)-1;
%   out(4)=out(4)+4;
%   out(2)=out(2)-1;
%   %out(2)=out(2)+9;
    
    if nargin <=1
        post_corr = zeros(size(out));
        post_corr(1)=-4;
        post_corr(2)=-1;
        post_corr(3)=-1;
        post_corr(4)=4;
    end
  
    out=out+post_corr;

end