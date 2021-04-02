function [m, err] = rad_dist_remove(m_d, K, k)
% RAD_DIST_REMOVE Return the true pixel coordinates from the distorted ones
% (solving a non-linear iterative LS problem).
% 
%   m = RAD_DIST_REMOVE(m_d, K, k)
%
%   Input arguments:
%   ------------------
%   m_d:    Nx2 array, distorted image points
%   K:      intrisics matrix of the camera (Matlab convention)
%   k:      radial distortion coefficients of the camera
%
%   Output arguments:
%   ------------------
%   m:      Nx2 array, undistorted image points
%   err:    final error of the iterative solver (RMS value)
%
% See also RAD_DIST_APPLY

    % A maximum of 3 distortion coefficients is considered
    k = [k(:); zeros(3-length(k),1)];
    
    % Notation:
    % m_d = (u_d, v_d)
    % q_d = (x_d, y_d) = m_d*inv(K)
    % m = (u, v) = q*K
    % q = (x, y)
    
    % Convert to normalized coordinates
    q_d = hom_tf(m_d, inv(K));
    
    % Retrieve the (normalized) true coordinates
    q = zeros(size(q_d));
    errs = zeros(numel(q_d),1);
    for i=1:size(q_d,1)

        % Optimization objective (function handle)
        fn_obj = @(q) rad_dist_remove_objective(q, q_d(i,:), k);

        % Optimization initial point
        q0 = q_d(i,:);
        % q0 = [0 0];

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

        % Launch optimization (radial distortion removal)
        [qi,~,err_qi,~,~,~,J] = lsqnonlin(fn_obj, q0, [], [], options);
        
        % Store results
        q(i,:) = qi;
        errs(2*i-1:2*i) = err_qi;
        
    end
    
    % Retrieve the true coordinates
    % (x, y) --> (u, v)
    m = hom_tf(q, K);
    
    % Return the RMS error of the results
    err = rms(errs);

end

function [err, J] = rad_dist_remove_objective(q, q_d, k)
    
    % Notation:
    % m_d = (u_d, v_d)
    % q_d = (x_d, y_d) = m_d*inv(K)
    % m = (u, v) = q*K
    % q = (x, y)

    % Distortion radius
    x = q(1);
    y = q(2);
    r2 = x^2 + y^2;

    % Distortion residual
    err = q(:) .* (1 + k(1)*r2 + k(2)*r2^2 + k(3)*r2^3) - q_d(:);

    % Derivative of res(1) wrt x
    d_res1_x = 1 + k(1)*r2 + k(2)*r2^2 + k(3)*r2^3 + ( 2*k(1) + 4*k(2)*r2 + 6*k(3)*r2^2 ) * x^2;
    % Derivative of res(1) wrt y
    d_res1_y = ( 2*k(1) + 4*k(2)*r2 + 6*k(3)*r2^2 ) * x * y;
    % Derivative of res(2) wrt x
    d_res2_x = ( 2*k(1) + 4*k(2)*r2 + 6*k(3)*r2^2 ) * x * y;
    % Derivative of res(2) wrt y
    d_res2_y = 1 + k(1)*r2 + k(2)*r2^2 + k(3)*r2^3 + ( 2*k(1) + 4*k(2)*r2 + 6*k(3)*r2^2 ) * y^2;

    % Build the Jacobian of res wrt x,y
    J = [
        d_res1_x   d_res1_y
        d_res2_x   d_res2_y
    ];

end