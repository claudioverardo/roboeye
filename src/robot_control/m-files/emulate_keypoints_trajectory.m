function trajectory_robot = emulate_keypoints_trajectory(start, trajectory)
% EMULATE_KEYPOINTS_TRAJECTORY Given a trajectory defined via keypoints
% return the actual trajectory followed by the robot. The actual trajectory
% is interpolated by the microcontroller with braccioServoMovement(...).
%
%   Input arguments:
%   ------------------
%   start:              1xQNUM array, starting point of the trajectory
%   trajectory:         NxQNUM array, trajectory defined by keypoints
%
%   Output arguments:
%   ------------------
%   trajectory_robot:   MxQNUM array, interpolated trajectory (M>=N)
%
% See also GENERATE_TRAJECTORY
    
    QNUM = size(trajectory,2);
    trajectory_robot = [];
    
    % For each couple of points of the keypoints trajectory
    for i=1:size(trajectory,1)
        
        if i==1
            current = round(start);
        else
            current = round(trajectory(i-1,:));
        end
        
        target = round(trajectory(i,:));
        
        n_points_pred = max(abs(current-target));
        trajectory_robot_i = zeros(n_points_pred,QNUM);
        
        % Emulate the interpolation performed by braccioServoMovement on Arduino
        for j=1:n_points_pred
            
            step = zeros(1,QNUM);
            step(current<target) = 1;
            step(current>target) = -1;
            
            current = current + step;
            trajectory_robot_i(j,:) = current;
            
        end
        
        % Store the emulated trajectory
        trajectory_robot = [trajectory_robot; trajectory_robot_i];
        
    end
    
    % If starting point and ending point of the trajectiory are the same the
    % controller still have to process one keypoint
    if size(trajectory_robot,1) == 0
        trajectory_robot = round(start);
    end

end