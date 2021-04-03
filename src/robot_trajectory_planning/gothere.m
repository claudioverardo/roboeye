function [qrob,errorflag,q] = gothere(braccio_params,x,y,z,roll,grasp,offset,varargin)
%Returns joint's angular position for a given spacial position in phisical
%space
%   the function decides autonomously the end effector orientation in order
%   to manage to reach the spatial point in the biggest workspace possible

    % Default values of parameters    
    default_verbose = 0;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'verbose', default_verbose, @(x) x>=0);
    parse(p, varargin{:});
    
    % Parse function parameters
    VERBOSE = p.Results.verbose;

    % Offset correction
    braccio_params(4)=braccio_params(4)+offset;

    % Initialization flags
    errorflag=false;

    % Define translation
    transl=[x y z];

    if x>=0 % Frontal workspace
        dirindex=-1;
    else % Backward workspace
        dirindex=1;
    end

    [qrob,q,foundflag] = scan_EF_pitch(transl,dirindex,braccio_params);

    % Check if the algorithm have found a solution and if not check the other direction
    if foundflag==false 
        dirindex=-dirindex;
        [qrob,q,foundflag] = scan_EF_pitch(transl,dirindex,braccio_params);
    end

    % Check if no solution was found
    if foundflag==false 
        errorflag=true;
        % disp("------no solution found------")
    else
        % disp("------solution found------")
    end
    
    if VERBOSE > 0
        jp=plot_config_rob(qrob,braccio_params);
        jp=plot_config([0 q],braccio_params);
    end

    % Set grabber angle
    qrob(5)=roll;
    qrob(6)=grasp;
    
end

