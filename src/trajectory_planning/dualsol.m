function [qlocdual] = dualsol(qloc)
%dualsol calculate dual solution of joint configuration
%   Detailed explanation goes here

   qlocdual=qloc;

   if qloc(3)*qloc(2)>=0
       qlocdual(2)=qloc(2)-qloc(3);
       qlocdual(3)=-qloc(3);
       qlocdual(4)=qloc(4)-qloc(3);
   else
       qlocdual(2)=qloc(2)+qloc(3);
       qlocdual(3)=-qloc(3);
       qlocdual(4)=qloc(4)+qloc(3);
   end

end

