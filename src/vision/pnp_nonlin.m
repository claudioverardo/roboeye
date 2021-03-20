function [R, t, reproj_err] = pnp_nonlin(R0, t0, X_image, X_world, K)
% PNP_NONLIN Refinement of Perspective-n-Points (PnP) from 3D-2D correspondences.
%
%   [R, t] = PNP_NONLIN(R0, t0, X_image, X_world, K) refines the input camera 
%   pose R0, t0 from a set of 2D-3D correspondences defined by X_image, X_world 
%   respectively. The algorithm minimizes the reprojection errors.
%
%   [R, t, reproj_err] = PNP_NONLIN(R0, t0, X_image, X_world, K) return also
%   the RMS value of the reprojection errors of the 3D-2D correspondences.
%
%   Input arguments:
%   ------------------
%   R0:         Initial rotation matrix for the non-linear iterative method,
%               typically calculate through the pnp_lin function
%   t0:         Initial translate vector for the non-linear iterative method,
%               typically calculate through the pnp_lin function
%   X_image:    Nx2 array, 2D image points
%   X_world:    Nx3 array, 3D world points
%   K:          Intrisics matrix of the input camera
%
%   Output arguments:
%   ------------------
%   R:          rotation matrix 3x3 (Matlab convetion)
%   t:          translate vector 1x3 (Matlab convetion)
%   reproj_err: reprojection error (RMS value)
%
%   NOTE: points and K with Matlab conventions, X_image = X_world*[R; t]*K.
%
%   See also PNP_LIN, REPROJECTION_ERROR
    
    % Transform world point in order to have the axes origin at R0,t0
    % X_world0 = X_world * T0
    X_world0 = homography (X_world, [R0, zeros(3,1); t0 1]);
    
    % Optimization objective (function handle)
    % x = [roll, pitch, yaw, tx, ty, tz] is the optimization variable
    % cf. rpy2rot() for the parameterization of R with roll, pitch, yaw
    fn_obj = @(x) pnp_nonlin_objective(x, X_image, X_world0, K);
    
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
        'Display', 'none', ... % debugging 'iter'
        'FiniteDifferenceType', 'central' ...
    );
    
    % Launch optimization (PnP refinement)
    [x,~,reproj_errs,~,~,~,J] = lsqnonlin(fn_obj, x0, [], [], options);
    R_delta = rpy2rot(x(1:3));
    t_delta = x(4:6);
    
    % Return the refined pose in world coordinates
    R = R0*R_delta;
    t = t0*R_delta + t_delta;
    
    % Return the overall reprojection error (RMS value)
    reproj_err = rms(reproj_errs);
    
    % PnP refinement with Fusiello ComputerVisionToolkit (debug)
    % [R, t] = exterior_nonlin(R, t, X_image', X_world', K');
    % R = R';
    % t = t';

end

function [err,J] = pnp_nonlin_objective(x, m, M, K)
    
    % Retrieve the actual pose [R;t] (under optimization)
    [R, J_roll, J_pitch, J_yaw] = rpy2rot(x(1:3));
    t = x(4:6);
    
    % Retrieve the Jacobians of R_ij wrt roll, pitch, yaw
    J_R = [J_roll(:), J_pitch(:), J_yaw(:)];
    
    % Reprojection error and Jacobian of each 3D-2D correspondence
    err = [];
    J =[];
    for i = 1:size(M,1)
        
        % i-th 3D-2D correspondence
        M_i = M(i,:);
        m_i = m(i,:);
        
        % Reprojection error and Jacobian wrt external parameters
        % external parameters = [R11,R21,R31,R12,R22,R32,R13,R23,R33,t1,t2,t3]
        [err_i, J_ext_i] = reprojection_error(m_i, M_i, K, R, t);
        
        % Jacobian wrt x=[roll, pitch, yaw, t1, t2, t2]
        J_i = J_ext_i * blkdiag(J_R, eye(3));
        
        % Accumulate results
        J = [J; J_i];
        err = [err; err_i];

    end
    
end
