function target_q_fix = fix_target_q(target_q, current_q, last_q)
% FIX_TARGET_Q Add a small overshoot to the trajectory of the first joint
% during the movement to a target position.
%   
%   target_q_fix = FIX_TARGET_Q(target_q, current_q, last_q)
%
%   Input arguments:
%   ------------------
%   target_q:       1xQNUM array, target position in joints space
%   current_q:      1xQNUM array, current position in joints space
%   last_q:         1xQNUM array, last position in joints space
%
%   Output arguments:
%   ------------------
%   target_q_fix:   3xQNUM array, fixed target position
%
%   See also GENERATE_TRAJECTORY

    QNUM = numel(current_q);
    
    if nargin < 3
        last_q = current_q;
    end
    
    if     target_q(1) > current_q(1) && current_q(1) >= last_q(1)
        offset = 5;
    elseif target_q(1) > current_q(1) && current_q(1) <  last_q(1)
        offset = 5;
    elseif target_q(1) < current_q(1) && current_q(1) <= last_q(1)
        offset = -5;
    elseif target_q(1) < current_q(1) && current_q(1) >  last_q(1)
        offset = -5;
    else 
        offset = 0;
    end
    
    if offset ~= 0
        target_q_fix = zeros(3,QNUM);
        target_q_fix(1,:) = [target_q(1)+offset, current_q(2:QNUM)];
        target_q_fix(2,:) = [target_q(1), current_q(2:QNUM)];
        target_q_fix(3,:) = target_q;
    else
        target_q_fix = target_q;
    end

end