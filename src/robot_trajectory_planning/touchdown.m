function [Qrob, errorflag, Q_tot] = touchdown(braccio_params, x, y, z, post_corr, home, VERBOSE)
% TOUCHDOWN Function that computes a trajectory from the home position to a 
% target point. The trajectory is composed by two parts. The former arrives
% to a certain position above the target moving all the joints with constant
% velocities. The latter is a vertical path to the target point that keeps 
% the end effector orientation fixed.   
%
%   [Qrob, errorflag] = TOUCHDOWN(braccio_params, x, y, z, post_corr, home,
%   VERBOSE)
%
%   Input arguments:
%   ------------------
%   braccio_params:     1xQNUM-1 array, real parameters of the Braccio robot,
%                       cf. direct_kin(...)
%   x:                  target x-position of end effector (robot frame)
%   y:                  target y-position of end effector (robot frame)
%   z:                  target z-position of end effector (robot frame)
%   post_corr:          1xQNUM-1 array, offsets to be applied a posteriori
%                       cf. braccio_angles(...)
%   home:               1xQNUM array, home position of the robot
%   VERBOSE:            verbose level of the function
%                       - 0: show nothing
%                       - 1: show the trajectory
%
%   Output arguments:
%   ------------------
%   Qrob:               170xQNUM array, pointwise trajectory in the space 
%                       of joints (robot convention)
%   errorflag:          1 if for at least one of the keypoints either the
%                       solution does not satisfy the robot constraint or
%                       the fsolve routine fails, 0 otherwise
%   Q_tot:              170xQNUM array, pointwise trajectory in the space 
%                       of joints (model convention)
%
%   NOTE: do not use z too high (remain in <= 40mm), stay in the range
%   140<=r<=360 mm where r=sqrt(x^2+y^2).
%
% See also GENERATE_TRAJECTORY

    errorflag=0;


    rmax=360;
    rmin=150;

    npoints=20;
    npoints_first=150;
    startingpos=[0 0 0 0 0];

    vertical_run=10;
    eulr3=0;
    grabberangle =ones(npoints,1)*72;

    Q=zeros(npoints,6);
    Qrob=zeros(npoints,6);
    r=sqrt(x^2+y^2);

    if r<rmin || r>rmax
        disp('####### ERROR: POSITION OUT OF RANGE #######')
        errorflag = 1;
    else
        %compute end effector angle by linear interpolation
        eulr2 = 175-(r-rmin)/(rmax-rmin)*45;
        

        %generate points in workspace 
        X=[ones(npoints,1)*x ones(npoints,1)*y linspace(z+vertical_run,z,npoints)' ones(npoints,1)*eulr2 ones(npoints,1)*eulr3];

        for i=1:npoints

           %traslation
           transl=X(i,[1 2 3]);
           %euler angles
           eulr=[atan2(transl(2),transl(1)) X(i,[4 5])*pi/180];
           %solve inverse kinematics
           [qloc, fval, info] = inverse_kin_simple(transl,eulr,startingpos,braccio_params);
           qloc=mod(qloc+180,360)-180;

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



           Q(i,[1 2 3 4 5])=qloc;
           %vector with [squared error, solver flag]
           controlvec(i,:)=[sum(sum(fval.^2)) info];



           %set grabber angle
           Q(i,6)=grabberangle(i);
           startingpos=Q(i,[1 2 3 4 5]);
        end

        %generate first part of trajectory
        Q_first=[linspace(0,Q(1,1),npoints_first)'...  %M1
                 linspace(0,Q(1,2),npoints_first)'...  %M2
                 linspace(0,Q(1,3),npoints_first)'...  %M3
                 linspace(0,Q(1,4),npoints_first)'...  %M4
                 linspace(0,Q(1,5),npoints_first)'...  %M5
                 linspace(10,Q(1,6),npoints_first)'...  %M6
                 ];

        Q_tot=[Q_first; Q];

        for i=1:length(Q_tot(:,1))
            Qrob(i,:)=[braccio_angles(Q_tot(i,1:end-1),post_corr,home(1:end-1)) Q_tot(i,6)];
            if ~check_limits_joints(Qrob(i,:))
                errorflag=1;
            end
        end

        for i=1:length(controlvec(:,2))
            if controlvec(i,2)<=0
                errorflag=1;
            end
        end

        if VERBOSE > 0
            jointpos=plot_config(Q_tot(:,1:5), braccio_params);
            jointpos=plot_config_rob(Qrob(:,1:5), braccio_params, post_corr, home);
        end

        %disp(eulr2)
        %maxexcurtion=max(Qrob);

    end
    
end

