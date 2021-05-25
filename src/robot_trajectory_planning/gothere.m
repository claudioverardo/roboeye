function [qrob, errorflag, q] = gothere(braccio_params, x, y, z, roll, grasp, offset, q_pre, post_corr, home, varargin)
% GOTHERE Returns the angular positions of the joints for a given spatial  
% position of the end effector. The function explores all the configurations
% of the 4th joint and it finds the first one that satisfy inverse kinematics. 
% The rationale behind this choice is to decide autonomously the end effector
% orientation in order to reach positions in the largest workspace possible.
% Moreover, if a previous position of the robot is provided, the algorithm
% choose the solution that is closest to it in joints space.
%
%   [qrob, errorflag, q] = GOTHERE(braccio_params, x, y, z, roll, grasp, 
%   offset, q_pre, post_corr, home)
%
%   Input arguments:
%   ------------------
%   braccio_params: 1xQNUM-1 array, real parameters of the Braccio robot,
%                   cf. direct_kin(...)
%   x:              target x-position of end effector (robot frame)
%   y:              target y-position of end effector (robot frame)
%   z:              target z-position of end effector (robot frame)
%   roll:           target position of the 5th joint (roll)
%   grasp:          target position of the 6th joint (gripper)
%   offset:         offset along z-axis of the 5th joint frame origin
%   q_pre:          1xQNUM array, previous position of the robot (optional)
%   post_corr:      1xQNUM-1 array, offsets to be applied a posteriori
%                   cf. braccio_angles(...)
%   home:           1xQNUM array, home position of the robot
%
%   Parameters:
%   --------
%   'verbose':      verbose level of the function (0, 1)
%                   - 0: show nothing
%                   - 1: show the solution found
%
%   Output arguments:
%   ------------------
%   qrob:           1xQNUM array, target position of joints (robot convention)
%   errorflag:      1 if either the solution does not satisfy the robot
%                   constraints or the fsolve routine fails, 0 otherwise
%   q:              1xQNUM array, target positions of "encoders" (debugging)
%
% See also GENERATE_TRAJECTORY, INVERSE_KIN_SUPER_SIMPLE

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

    % Initialization
    errorflag=false;
    q=[];
    qrob=[];

    % Define translation
    transl=[x y z];
    
    if isempty(q_pre)
        if x>=0 % Frontal workspace
            dirindex=-1;
        else % Backward workspace
            dirindex=1;
        end
        [qrob,q,foundflag] = scan_EF_pitch(transl,dirindex,braccio_params,post_corr,home);
        

        % Check if the algorithm have found a solution and if not check the other direction
        if foundflag==false 
            dirindex=-dirindex;
            [qrob,q,foundflag] = scan_EF_pitch(transl,dirindex,braccio_params,post_corr,home);
        end
    else
        %%%%% Search nearest solution
        Q_poss = all_config(transl,braccio_params,post_corr,home);
        
        if isempty(Q_poss)
            foundflag=false;
        else
            foundflag=true;

            dist=vecnorm(Q_poss-q_pre,2,2);

            [~,pos_min]=min(dist);

            qrob=Q_poss(pos_min,:);
            q=braccio_angles_inv(qrob(1:5),post_corr,home(1:end-1));
        end
    end
    
    % Check if no solution was found
    if foundflag==false 
        errorflag=true;
        % disp("------no solution found------")
    else
        % disp("------solution found------")
    end
    
    if VERBOSE > 0
        jp=plot_config_rob(qrob(1:5),braccio_params,post_corr,home);
        jp=plot_config(q(1:5),braccio_params);
    end

    % Set grabber angle
    qrob(5)=roll;
    qrob(6)=grasp;
    
end

function Q_poss = all_config(transl,braccio_params,post_corr,home)
%calculate all joints configurations varing q4
   
   startingpos=[0 0 0 0 0];
   
   index=0;

   q4_aux=braccio_angles_inv([0 0 0 180 0],post_corr,home(1:end-1));
   q4_min=q4_aux(4);
   q4_aux=braccio_angles_inv([0 0 0 0 0],post_corr,home(1:end-1));
   q4_max=q4_aux(4);
   
   q4_poss=[q4_min:1:q4_max];
   Q_poss_fix=zeros(length(q4_poss),6);

    for q4=q4_poss
        errorflag=false;
        
        %solve inverse kinematics
        %[qloc, fval, info] = inverse_kin_super_simple(transl,q4,startingpos,braccio_params);
        [qloc, info] = inverse_kin_super_simple_an(transl,q4,braccio_params);
        fval = 0;
        qloc=mod(qloc+180,360)-180;

        %corrections
        corr=z_correction(qloc,transl);
        qloc=qloc+corr;

        %check if solver failed
        if info<=0
            errorflag=true;
        end

           %check for dual solution:
           if transl(1) >= 0
               if qloc(3)>0
                    qloc=dualsol(qloc);
               end
           else
               if qloc(3)<0
                    qloc=dualsol(qloc);
               end
           end


        %convert joint pos into robot's rf
        qrob=[braccio_angles(qloc,post_corr,home(1:end-1)) 0];    
        if ~check_limits_joints(qrob) %if the solution is not in the range check for dual
           qloc=dualsol(qloc);
           qrob=[braccio_angles(qloc,post_corr,home(1:end-1)) 0];
           if ~check_limits_joints(qrob) %if even dual is out of range trigger error flag
               errorflag=true;
           end    
        end

        if errorflag==false
            index=index+1;
            Q_poss_fix(index,:)=qrob;
        end

    end
    Q_poss=Q_poss_fix([1:1:index],:);
    %n_el=index;    
end

function [qrob,q_teo,foundflag] = scan_EF_pitch(transl,dirindex,braccio_params,post_corr,home)
%Find a suitable EF angle in one direction (dirindex)
    exitflag=false;
    foundflag=false;
    
    %eulr=[atan2(transl(2),transl(1)) 0 0];
    eulr_index=-1;
    startingpos=[0 0 0 0 0];
    q=[0 0 0 0 0 0];
    cycle=0;

    while exitflag == false
        cycle=cycle+1;
        errorflag=false;


        %increment EF angle
        eulr_index=eulr_index+1;
        joint4=eulr_index*dirindex;

        %solve inverse kinematics
        %[qloc, fval, info] = inverse_kin_super_simple(transl,joint4,startingpos,braccio_params);
        [qloc, info] = inverse_kin_super_simple_an(transl,joint4,braccio_params);
        fval = 0;
        qloc=mod(qloc+180,360)-180;
        %disp(qloc(4));
        %q([1 2 3 4 5])=qloc;
        
        %corrections
        corr=z_correction(qloc,transl);
        qloc=qloc+corr;

        %check for solver errors
        controlvec=[sum(sum(fval.^2)) info];

        %check if solver failed
        if controlvec(2)<=0
            errorflag=true;
        end
        
        %check for dual solution:
        if transl(1) >= 0
           if qloc(3)>0
                qloc=dualsol(qloc);
           end
        else
           if qloc(3)<0
                qloc=dualsol(qloc);
           end
        end

        %convert joint pos into robot's rf
        qrob=[braccio_angles(qloc,post_corr,home(1:end-1)) q(6)];    
        if ~check_limits_joints(qrob) %if the solution is not in the range check for dual
           qloc=dualsol(qloc);
           qrob=[braccio_angles(qloc,post_corr,home(1:end-1)) q(6)];
           if ~check_limits_joints(qrob) %if even dual is out of range trigger error flag
               errorflag=true;
           end    
        end
        
        % define q in algorithm rf
        q_teo([1 2 3 4 5])=qloc-corr;
        

        if errorflag==false
            exitflag=true;
            foundflag=true;
        end
        
        if eulr_index>99
            exitflag=true;
        end
        
    end
end