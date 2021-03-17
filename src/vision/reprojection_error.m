function [err, J_ext] = reprojection_error(K, R, t, m, M)
% REPROJECTION_ERROR Reprojection error and jacobian for one 3D-2D correspondence
% 
% NOTE: points and matrices with Fusiello convention
    
    M_hom = [M;1];   % make homogeneous
    
    % Hereafter, fp denotes the perspective division.
    % It converts homogeneous coordinates to inhomogenous coordinates
    % fp([X,Y,Z]) = [X/Z, Y/Z]
    
    % reprojection of the input 3D points to 2D normalized points (homogeneous coordinates)
    q_proj_hom = [R t] * M_hom;   
    
    % reprojection of the input 3D points to 2D points (inhomogeneous coordinates)
    % NOTE: m_proj = fp(K*[R t]*M_hom) = fp(K*q_proj_hom)
    m_proj = K * q_proj_hom;
    m_proj = m_proj(1:2) ./ m_proj(3);
    
    % Calculate reprojection error
    err = m_proj(:) - m(:);
    
    % Jacobian of the perspective division computed in [X,Y,Z] = q_proj
    % J_fp = [1/Z 0 -X/Z^2; 0 1/Z -Y/Z^2]
    J_fp = [
        1/q_proj_hom(3), 0, -q_proj_hom(1)/q_proj_hom(3)^2; 
        0, 1/q_proj_hom(3), -q_proj_hom(2)/q_proj_hom(3)^2;
    ];
    
    % Jacobian of the reprojected points wrt external parameters R,t
    % m_proj = K * [fp(q_proj_hom); 1]
    % J_ext = K(1:2,1:2) * J( fp(q_proj_hom)]
    J_ext = K(1:2,1:2) * J_fp * kron( M_hom', eye(3) );

end