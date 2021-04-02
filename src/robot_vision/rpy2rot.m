function [R, J_roll, J_pitch, J_yaw] = rpy2rot(a)
% RPY2ROT Create a rotation matrix from its roll-pitch-yaw parameterization.
%
%   [R, J_roll, J_pitch, J_yaw] = RPY2ROT(a)
%
%   Input arguments:
%   ------------------
%   a:          [roll pitch yaw] parameterization of the rotation
%               - a(1) = roll,  rotation angle around x-axis
%               - a(2) = pitch, rotation angle around y-axis
%               - a(3) = yaw,   rotation angle around z-axis
%
%   Output arguments:
%   ------------------
%   R:          rotation matrix, R = Rx(roll)*Ry(pitch)*Rz(yaw)
%   J_roll:     Jacobian of R wrt roll
%   J_pitch:    Jacobian of R wrt pitch
%   J_yaw:      Jacobian of R wrt yaw
    
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