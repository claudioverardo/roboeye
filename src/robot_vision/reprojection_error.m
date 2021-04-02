function [err, J_ext] = reprojection_error(m, M, K, R, t, k)
% REPROJECTION_ERROR Reprojection error of a 3D-2D correspondence. It finds
% the component-wise reprojection error between a 2D point and a 3D point.
% The Jacobian wrt the extrinsics of the camera is also returned.
%
%   [err, J_ext] = REPROJECTION_ERROR(m, M, K, R, t, k)
%
%   Input arguments:
%   ------------------
%   m:      Nx2 array, 2D image points
%   M:      Nx3 array, 3D world points
%   K:      intrisics matrix of the camera
%   R:      rotation matrix of the camera extrinsics
%   t:      translation vector of the camera extrinsics
%   k:      radial distortion coefficients of the camera
%
%   Output arguments:
%   ------------------
%   err:    2Nx1 array, reprojection error between m and reproj(M)
%   J_ext:  2Nx12 array, Jacobian of err wrt the camera extrinsics
%           [R11,R21,R31,R12,R22,R32,R13,R23,R33,t1,t2,t3]
%
%   NOTE: Matlab convention is assumed, reproj(M) = fd( M*[R; t]*K ) where
%   fd is the function that applies the radial distortion.
%
%   See also PNP_LIN, PNP_NONLIN, RAD_DIST_APPLY
    
    % Hereafter, fp denotes the perspective division.
    % It converts homogeneous coordinates to inhomogenous coordinates
    % fp([X,Y,Z]) = [X/Z, Y/Z]
    
    % Moreover, fd denotes the radial distortion function.
    % It converts the true image points [px] into the distorted ones [px]
    % fd([u,v]) = [u_d, v_d]
    
    n_points = size(M,1);
    
    % Reprojection of the input 3D points (normalized homogeneous coordinates)
    M_hom = [M, ones(n_points,1)];
    q_proj_hom =  M_hom * [R; t];   
    
    % Reprojection of the input 3D points (inhomogeneous coordinates)
    % m_proj = fp(M_hom*[R;t]*K) = fp(q_proj_hom*K)
    m_proj = q_proj_hom * K;
    m_proj = m_proj(:,1:2) ./ m_proj(:,3);
    
    % Apply radial distortion
    % m_proj_d = fd(m_proj)
    if nargin > 5
        [m_proj_d, J_fd] = rad_dist_apply(m_proj, K, k);
    % Or ignore it
    else 
        m_proj_d = m_proj;
        J_fd = cell(1,n_points);
        J_fd(:) = { eye(2) };
    end
    
    % Calculate the component-wise reprojection error
    err = m_proj_d - m;
    err = reshape(err',[],1);
    
    % Jacobian of the error wrt the extrinsics parameters
    % extrinsics parameters = [R11,R21,R31,R12,R22,R32,R13,R23,R33,t1,t2,t3]
    if nargout > 1
        
        J_ext = [];
        for i=1:size(M,1)
            
            % Jacobian of M*[R;t]*K wrt extrinsics parameters computed in M
            J_Rt_i = [kron(eye(3), M(i,:)), eye(3)];
            % J_Rt_i = kron( M_hom(i,:), eye(3) ); % Jacobian wrt R', not R

            % Jacobian of the perspective division fp wrt [X,Y,Z] computed in q_proj_hom
            % NOTE: [X Y Z] is the argument of the perspective division fp
            %       J_fp = [1/Z 0 -X/Z^2; 0 1/Z -Y/Z^2]
            J_fp_i = [
                1/q_proj_hom(i,3), 0, -q_proj_hom(i,1)/q_proj_hom(i,3)^2; 
                0, 1/q_proj_hom(i,3), -q_proj_hom(i,2)/q_proj_hom(i,3)^2;
            ];
        
            % Jacobian of the de-normalization function wrt normalized coordinates (constant)
            % m_proj = norm2px(q_proj)
            J_norm2px = K(1:2,1:2)';

            % Jacobian of the radial distortion function fd wrt (u,v) computed in m_proj_d
            % NOTE: [u v] is the argument of the radial distortion function fp
            %       J_fd{i} -> cf. radial_dist(...)

            % Jacobian of the reprojected points wrt extrinsics parameters computed in m_proj_d
            % m_proj = fd ( norm2px ( fp( K*[R, t]*M ) ) )
            J_ext_i = J_fd{i} * J_norm2px * J_fp_i * J_Rt_i; % ... chain rule
        
            J_ext = [J_ext; J_ext_i];
            
        end
        
    end

end