function [R, J_roll, J_pitch, J_yaw] = rpy2rot(a)
% RPY2ROT Create rotation matrix from roll-pitch-yaw parameterization
%
% a(1) = roll  (x-axis)
% a(2) = pitch (y-axis)
% a(3) = yaw   (z-axis)
%
% rotation matrix:
% R = Rx(roll)*Ry(pitch)*Rz(yaw)
    
    % Rotation around x-axis (roll)
    Rx =  [
        1       0          0
        0 cos(a(1)) -sin(a(1))
        0 sin(a(1))  cos(a(1))
    ];
    
    % Rotation around y-axis (pitch)
    Ry = [
        cos(a(2)) 0  sin(a(2))
              0   1        0
       -sin(a(2)) 0  cos(a(2))
    ];
    
    % Rotation around z-axis (yaw)
    Rz = [
        cos(a(3)) -sin(a(3)) 0
        sin(a(3))  cos(a(3)) 0
              0          0   1
    ];
    
    % Final rotation
    R = Rz * Ry * Rx;
    
    % If required, return the Jacobians of R
    if nargout > 1 
        
        % Derivative wrt roll angle
        J_roll  = Rz * Ry * [0 0 0; 0 -sin(a(1)) -cos(a(1)); 0 cos(a(1)) -sin(a(1))];
        % Derivative wrt pitch angle
        J_pitch = Rz * [-sin(a(2)) 0 cos(a(2)); 0 0 0; -cos(a(2)) 0 -sin(a(2))] * Rx;
        % Derivative wrt yaw angle
        J_yaw   = [-sin(a(3)) -cos(a(3)) 0; cos(a(3)) -sin(a(3)) 0; 0 0 0] * Ry * Rx;
        
    end
    
end