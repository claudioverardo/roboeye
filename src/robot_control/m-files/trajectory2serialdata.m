function data_tx = trajectory2serialdata(trajectory_type, delta_t, trajectory)
% TRAJECTORY2SERIALDATA Prepare trajectory data to be sent to Arduino.
%
%   data_tx = TRAJECTORY2SERIALDATA(trajectory_type, delta_T, trajectory)
%
%   Input arguments:
%   ------------------
%   type_trajectory: type of trajectory, cf. generate_trajectory(...)
%       - 'pointwise': trajectory defined point by point
%       - 'keypoints': trajectory defined via keypoints to be interpolated
%   delta_t: timestep of the trajectory execution
%   trajectory: NxQNUM array, points of the trajectory
%
%   Output arguments:
%   ------------------
%   data_tx: trajectory data in the format required by Arduino
%
%   See also ROBOT_FSM_INTERFACE

    switch trajectory_type
        case 'pointwise'
            trajectory_type_id = 1;
        case 'keypoints'
            trajectory_type_id = 2;
        otherwise
            trajectory_type_id = 0;
    end
    
    n_points_trajectory = size(trajectory,1);
    trajectory_vec = reshape(trajectory',1,[]);
    data_tx = uint8( [trajectory_type_id n_points_trajectory delta_t trajectory_vec] );

end