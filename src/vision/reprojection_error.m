function [err, J_ext] = reprojection_error(m, M, K, R, t)
% REPROJECTION_ERROR Reprojection error of a 3D-2D correspondence. It finds
% the component-wise reprojection error between a 2D point and a 3D point.
% The Jacobian wrt the extrinsics of the camera is also returned.
%
%   [err, J_ext] = REPROJECTION_ERROR(m, M, K, R, t)
%
%   Input arguments:
%   ------------------
%   m:      2D image point
%   M:      3D world point
%   K:      intrisics matrix of the camera
%   R:      rotation matrix of the camera extrinsics
%   t:      translation vector of the camera extrinsics
%
%   Output arguments:
%   ------------------
%   err:    2x1 array, reprojection error between m and reproj(M)
%   J_ext:  2x12 array, Jacobian of err wrt the camera extrinsics
%           [R11,R21,R31,R12,R22,R32,R13,R23,R33,t1,t2,t3]
%
%   NOTE: Matlab convention is assumed, reproj(M) = M*[R; t]*K.
%
%   See also PNP_LIN, PNP_NONLIN
    
    % Hereafter, fp denotes the perspective division.
    % It converts homogeneous coordinates to inhomogenous coordinates
    % fp([X,Y,Z]) = [X/Z, Y/Z]
    
    % Reprojection of the input 3D point (normalized homogeneous coordinates)
    M_hom = [M 1];
    q_proj_hom =  M_hom * [R; t];   
    
    % Reprojection of the input 3D points (inhomogeneous coordinates)
    % m_proj = fp(M_hom*[R;t]*K) = fp(q_proj_hom*K)
    m_proj = q_proj_hom * K;
    m_proj = m_proj(1:2) ./ m_proj(3);
    
    % Calculate the component-wise reprojection error
    err = m_proj(:) - m(:);
    
    % If required compute the Jacobian of the error wrt the external parameters
    % external parameters = [R11,R21,R31,R12,R22,R32,R13,R23,R33,t1,t2,t3]
    if nargout > 1

        % Jacobian of [X,Y,Z] wrt external parameters computed in M
        % [X Y Z] is the argument of the perspective division fp
        J_Rt = [kron(eye(3), M), eye(3)];
        % J_Rt = kron( M_hom, eye(3) ); % Jacobian wrt R', not R
    
        % Jacobian of the perspective division fp wrt [X,Y,Z] computed in q_proj_hom
        % J_fp = [1/Z 0 -X/Z^2; 0 1/Z -Y/Z^2]
        J_fp = [
            1/q_proj_hom(3), 0, -q_proj_hom(1)/q_proj_hom(3)^2; 
            0, 1/q_proj_hom(3), -q_proj_hom(2)/q_proj_hom(3)^2;
        ];

        % Jacobian of the reprojected points wrt external parameters computed in m_proj
        % m_proj = [fp(q_proj_hom) 1]*K
        % J_ext = K(1:2,1:2)' * J( fp(q_proj_hom) )      ... chain rule
        %       = K(1:2,1:2)' * J_fp(q_proj_hom) * J_Rt  
        J_ext = K(1:2,1:2)' * J_fp * J_Rt;
    
    end

end