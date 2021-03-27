function [R, t, reproj_err] = pnp_nonlin(R0, t0, X_image, X_world, K, k)
% PNP_NONLIN Non-linear refinement of Perspective-n-Points (PnP) from 3D-2D
% correspondences. It iterativelly refines the input camera extrinsics through
% the minimization of the reprojection errors of a set of 3D-2D correspondences.
% Also the RMS value of the final reprojection errors is returned.
%
%   [R, t, reproj_err] = PNP_NONLIN(R0, t0, X_image, X_world, K, k)
%
%   Input arguments:
%   ------------------
%   R0:         initial guess for the rotation matrix of the camera extrinsics,
%               e.g., calculated with pnp_lin(...)
%   t0:         initial guess for the translation vector of the camera extrinsics,
%               e.g., calculated with pnp_lin(...)
%   X_image:    Nx2 array, 2D image points
%   X_world:    Nx3 array, 3D world points
%   K:          intrisics matrix of the camera
%   k:          radial distortion coefficients of the camera
%
%   Output arguments:
%   ------------------
%   R:          rotation matrix of the (refined) camera extrinsics
%   t:          translation vector of the (refined) camera extrinsics
%   reproj_err: reprojection error (RMS value)
%
%   NOTE: Matlab convention is assumed, X_image = fd( X_world*[R; t]*K ) where
%   fd is the function that applies the radial distortion.
%
%   See also PNP_LIN, REPROJECTION_ERROR
    
    if nargin < 6
        k = []; % Ignore radial distortion
    end
    
    % Transform world point in order to have the camera as reference
    % X_world0 = X_world * T0
    X_world0 = hom_tf (X_world, [R0, zeros(3,1); t0 1]);
    
    % Optimization objective (function handle)
    % x = [roll, pitch, yaw, tx, ty, tz] is the optimization variable
    % cf. rpy2rot() for the parameterization of R with roll, pitch, yaw
    fn_obj = @(x) pnp_nonlin_objective(x, X_image, X_world0, K, k);
    
    % Optimization initial point 
    x0 = [0 0 0 0 0 0];

    % Optimization options
    MaxIterations = 200;
    FunctionTol = 1e-10;
    StepTol = 1e-6;
    options = optimoptions('lsqnonlin', ...
        'Algorithm', 'levenberg-marquardt', ...
        'SpecifyObjectiveGradient', true, ... % false for numeric gradients
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
    
    % Return the refined extrinsics in the world frame
    R = R0*R_delta;
    t = t0*R_delta + t_delta;
    
    % Return the overall reprojection error (RMS value)
    reproj_err = rms(reproj_errs);
    
    % PnP refinement with Fusiello ComputerVisionToolkit (debug)
    % [R, t] = exterior_nonlin(R, t, X_image', X_world', K');
    % R = R';
    % t = t';

end

function [err,J] = pnp_nonlin_objective(x, m, M, K, k)
    
    % Retrieve the actual extrinsics [R;t] (under optimization)
    [R, J_roll, J_pitch, J_yaw] = rpy2rot(x(1:3));
    t = x(4:6);
        
    % Reprojection error and its Jacobian wrt extrinsics parameters
    [err, J_ext] = reprojection_error(m, M, K, R, t, k);
    
    % Jacobian of extrinsics parameters wrt x
    % extrinsics parameters = [R11,R21,R31,R12,R22,R32,R13,R23,R33,t1,t2,t3]
    % x = [roll, pitch, yaw, tx, ty, tz]
    J_R = [J_roll(:), J_pitch(:), J_yaw(:)];
    J_x = blkdiag(J_R, eye(3));
    
    % Jacobian of err wrt x
    J = J_ext * J_x; % ... chain rule
    
end
