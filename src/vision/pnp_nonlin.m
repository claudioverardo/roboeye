function [R,t] = pnp_nonlin(R0, t0, X_image, X_world, K_obj)
% PNP_NONLIN Non linear refinement of Perspective-n-Points (PnP) from 3D-2D correspondences
%
% X_image: Mx2 array
% X_world: Mx3 array
%
% NB: points and K with Matlab convention, X_image = X_world*[R;t]*K

    K = K_obj.IntrinsicMatrix;
    
    % Transfor world coordinates in order to have the axes origin at R0,t0
    % X_world0 = X_world * T0
    X_world0 = homography (X_world, [R0, zeros(3,1); t0 1]);
    
    % Optimization residual (function handle)
    % x = [roll, pitch, yaw, tx, ty, tz] is the optimization variable
    % cf. rpy2rot() for the parameterization of R with roll, pitch, yaw
    fn_res = @(x) pnp_nonlin_objective(x, X_image, X_world0, K);
    
    % Optimization initial point 
    x0 = [0 0 0 0 0 0];

    % Optimization options
    MaxIterations = 200;
    FunctionTol = 1e-10;
    StepTol = 1e-6;
    options = optimoptions('lsqnonlin', ...
        'Algorithm', 'levenberg-marquardt', ...
        'SpecifyObjectiveGradient', true, ...
        'MaxIterations' ,MaxIterations, ...
        'StepTolerance', StepTol, ...
        'FunctionTolerance', FunctionTol, ...
        'CheckGradients', false, ... % true to check the jacobians
        'Display', 'off', ... % debugging 'iter'
        'FiniteDifferenceType', 'central' ...
    );
    
    % Launch optimization (PnP refinement)
    [x,~,res,~,~,~,J] = lsqnonlin(fn_res, x0, [], [], options);
    R_delta = rpy2rot(x(1:3));
    t_delta = x(4:6);
    
    % Retrieve the refined pose in world coordinates
    R = R0*R_delta;
    t = t0*R_delta + t_delta;

end

function [res,J] = pnp_nonlin_objective(x, m, M, K)
    
    % Retrieve the actual pose [R;t] (under optimization)
    [R, J_roll, J_pitch, J_yaw] = rpy2rot(x(1:3));
    t = x(4:6);
    
    % Retrieve the Jacobians of R'
    % NOTE: reprojection_error() computes the jacobian wrt R' (cf. below)
    J_roll  = J_roll';
    J_pitch = J_pitch';
    J_yaw   = J_yaw';
    J_R = [J_roll(:), J_pitch(:), J_yaw(:)];
    
    % Reprojection errors and jacobians of each 3D-2D correspondence
    res = []; J =[];
    for i = 1:size(M,1)
        
        % i-th 3D-2D correspondence
        M_i = M(i,:);
        m_i = m(i,:);
        
        % Reprojection error and jacobian wrt external parameters [R',t']
        % NOTE: reprojection_error() adopts the Fusiello convention
        [res_i, J_ext_i] = reprojection_error(K', R', t', m_i', M_i');
        
        % Jacobian wrt [alpha, beta, gamma, t1, t2, t2]
        J_i = J_ext_i * blkdiag(J_R, eye(3));
        
        % Accumulate results
        J = [J; J_i];
        res = [res; res_i];

    end
    
end
