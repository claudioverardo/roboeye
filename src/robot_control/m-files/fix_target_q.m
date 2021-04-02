function target_q_fixed = fix_target_q(target_q, current_q, last_q)

    QNUM = numel(current_q);
    
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