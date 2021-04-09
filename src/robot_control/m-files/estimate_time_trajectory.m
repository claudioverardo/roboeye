function time = estimate_time_trajectory(type_trajectory, trajectory, current_q, delta_t)
% ESTIMATE_TIME_TRAJECTORY Estimate the time to execute a trajectory on the robot.
%
%   time = ESTIMATE_TIME_TRAJECTORY(type_trajectory, trajectory, current_q, delta_t)
%
%   Input arguments:
%   ------------------
%   type_trajectory: type of trajectory, cf. generate_trajectory(...)
%       - 'pointwise': trajectory defined point by point
%       - 'keypoints': trajectory defined via keypoints to be interpolated
%   trajectory: NxQNUM array, points of the trajectory
%   current_q: 1xQNUM array, current position of the robot (joints)
%   delta_t: timestep of the trajectory execution
%
%   Output arguments:
%   ------------------
%   time: estimated execution time of the trajectory
%
%   See also GENERATE_TRAJECTORY
        
    switch type_trajectory
        
        case 'pointwise'
            time = ceil( size(trajectory,1)*delta_t ./ 1000 + 0.1 );
    
        case 'keypoints'
            steps = diff(double([current_q; trajectory]));
            max_steps = max(abs(steps),[],2);
            time = ceil( sum(max_steps)*delta_t ./ 1000 + 0.1 );
        
    end
    
end