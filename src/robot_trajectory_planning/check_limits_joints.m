function check_ans = check_limits_joints(qrob)
% CHECK_LIMITS_JOINTS Check if a given position in the joints space satisfy
% the constraints of the Braccio robot.
%
%   check_ans = CHECK_LIMITS_JOINTS(qrob)
%
%   Input arguments:
%   ------------------
%   qrob:       1xQNUM array, joints position under test 
%
%   Output arguments:
%   ------------------
%   check_ans:  1 if qrob satisfy the constraints, 0 otherwise
%
% See also GOTHERE, TOUCHDOWN

    % define robot agles limits
    ang_pos_rob_max=[180 165 180 180 180 73];
    ang_pos_rob_min=[0 15 0 0 0 0];
    
    check_ans = all(qrob>=ang_pos_rob_min) && all(qrob<=ang_pos_rob_max);

end