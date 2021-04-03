function target_q_fixed = fix_target_q(target_q, current_q, last_q)
% FIX_TARGET_Q TODO
%
%   
%   target_q_fixed = FIX_TARGET_Q(target_q, current_q, last_q)
%
%   Input arguments:
%   ------------------
%   target_q:
%   current_q:
%   last_q:
%
%   Output arguments:
%   ------------------
%   target_q_fixed:   
%
%   See also TODO

    QNUM = numel(current_q);
    
    if nargin < 3
        last_q = current_q;
    end
    
    if     target_q(1) > current_q(1) && current_q(1) >= last_q(1)
        offset = 15;
    elseif target_q(1) > current_q(1) && current_q(1) <  last_q(1)
        offset = 15;
    elseif target_q(1) < current_q(1) && current_q(1) <= last_q(1)
        offset = -15;
    elseif target_q(1) < current_q(1) && current_q(1) >  last_q(1)
        offset = -15;
    else 
        offset = 0;
    end
    
    if offset ~= 0
        target_q_fixed = zeros(3,QNUM);
        target_q_fixed(1,:) = [target_q(1)+offset, current_q(2:QNUM)];
        target_q_fixed(2,:) = [target_q(1), current_q(2:QNUM)];
        target_q_fixed(3,:) = target_q;
    else
        target_q_fixed = target_q;
    end

end