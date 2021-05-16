function qlocdual = dualsol(qloc)
% DUALSOL For a given joints position in model convention, find the other one
% that preserves the end effector position and orientation ('dual position').
%
%   qlocdual = DUALSOL(qloc)
%
%   Input arguments:
%   ------------------
%   qloc:       1xQNUM-1 array, input joints position
%
%   Output arguments:
%   ------------------
%   qlocdual:   1xQNUM-1 array, dual position of qloc
%
% See also GOTHERE, TOUCHDOWN

   qlocdual=qloc;

%    if qloc(3)<=0
%        qlocdual(2)=qloc(2)-qloc(3);
%        qlocdual(3)=-qloc(3);
%        qlocdual(4)=qloc(4)-qloc(3);
%    else
%        qlocdual(2)=qloc(2)+qloc(3);
%        qlocdual(3)=-qloc(3);
%        qlocdual(4)=qloc(4)+qloc(3);
%    end

        qlocdual(2)=qloc(2)+qloc(3);
        qlocdual(3)=-qloc(3);
        qlocdual(4)=qloc(4)+qloc(3);

end

