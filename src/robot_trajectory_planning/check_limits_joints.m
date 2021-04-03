function check_ans = check_limits_joints(qrob)

    % define robot agles limits
    ang_pos_rob_max=[180 165 180 180 180 73];
    ang_pos_rob_min=[0 15 0 0 0 0];
    
    check_ans = all(qrob>=ang_pos_rob_min) && all(qrob<=ang_pos_rob_max);
    % any(qrob<ang_pos_rob_min) || any(qrob>ang_pos_rob_max)

end