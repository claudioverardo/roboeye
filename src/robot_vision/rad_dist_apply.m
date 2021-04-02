function [m_d, J_m] = rad_dist_apply(m, K, k)
% RAD_DIST_APPLY Return the distorted pixel coordinates from the true ones.
%
%   [m_d, J_m] = RAD_DIST_APPLY(m, K, k)
%
%   Input arguments:
%   ------------------
%   m:      Nx2 array, undistorted image points
%   K:      intrisics matrix of the camera (Matlab convention)
%   k:      radial distortion coefficients of the camera
%
%   Output arguments:
%   ------------------
%   m_d:    Nx2 array, distorted image points
%   J_m:    cell array of Jacobians of m_d wrt m (2x2 matrices)
%
% See also RAD_DIST_REMOVE

    % A maximum of 3 distortion coefficients is considered
    k = [k(:); zeros(3-length(k),1)];

    % Convert to normalized coordinates
    % m = (u, v)
    % q = (x, y) = m*inv(K)
    q = hom_tf(m, inv(K));
    x = q(:,1);
    y = q(:,2);
    
    % Compute the distortion radius
    r2 = x.^2 + y.^2;
    
    % Retrieve the (normalized) distorted coordinates
    % q_d = (x_d, y_d)
    q_d = q.*(1 + k(1)*r2 + k(2)*r2.^2 + k(3)*r2.^3);
    
    % Retrieve the distorted coordinates
    % m_d = (u_d, v_d) = q_d*K
    m_d = hom_tf(q_d, K);
    
    % Compute Jacobian of m_d = (u_d, v_d) wrt m = (u, v)
    if nargout > 1
            
        % fd: (x,y) -> (x_d,y_d) is the distortion function
        % J_fd = [d_xd_x, d_xd_y; d_yd_x, d_yd_y];
        
        % Derivatives of x_d wrt x (for each point)
        d_xd_x = 1 + k(1)*r2 + k(2)*r2.^2 + k(3)*r2.^3 + ( 2*k(1) + 4*k(2)*r2 + 6*k(3)*r2.^2 ) .* x.^2;
        % Derivatives of x_d wrt y (for each point)
        d_xd_y = ( 2*k(1) + 4*k(2)*r2 + 6*k(3)*r2.^2 ) .* x .* y;
        % Derivatives of y_d wrt x (for each point)
        d_yd_x = ( 2*k(1) + 4*k(2)*r2 + 6*k(3)*r2.^2 ) .* x .* y;
        % Derivatives of y_d wrt y (for each point)
        d_yd_y = 1 + k(1)*r2 + k(2)*r2.^2 + k(3)*r2.^3 + ( 2*k(1) + 4*k(2)*r2 + 6*k(3)*r2.^2 ) .* y.^2;
        
        % Build the Jacobians of m_d points wrt m points
        J_m = cell(0);
        for i = 1:size(m,1)
            
            % m_d = K*fd(inv(K)*m)
            % J_m(m_d) = K * J_fd * inv(K) ... chain rule
            J_m{i} = K(1:2,1:2)' * [
                d_xd_x(i)    d_xd_y(i)
                d_yd_x(i)    d_yd_y(i)
            ] * inv(K(1:2,1:2)');
            
        end
        
    end
    
end
