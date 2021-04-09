function qlocdual = dualsol(qloc)
% DUALSOL For a given joints position, find the other one that preserves the 
% end effector position and orientation ('dual position').
%
%   qlocdual = DUALSOL(qloc)
%
%   Input arguments:
%   ------------------
%   qloc:       input joints position
%
%   Output arguments:
%   ------------------
%   qlocdual:   dual position of qloc

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

