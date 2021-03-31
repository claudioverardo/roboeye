function [Qrob,errorflag,jointpos,controlvec,maxexcurtion,Q_tot] = touchdown(x,y,z)
% Makes trakectory to the point in input (with the last part vertical)
%   Do not use z to high (<= 40mm)
%   stay in the range 140<=r<=360 r=sqrt(x^2+y^2)

errorflag=0;

ang_pos_rob_max=[180 165 180 180 180 73];
ang_pos_rob_min=[0 15 0 0 0 0];


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
       [qloc, fval, info] = inverse_kin_simple(transl,eulr,startingpos);
       qloc=mod(qloc+180,360)-180;
       
       %check for dual solution:
       qloc(3)*qloc(2)
       if qloc(3)*qloc(2)<=0
            %disp('IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII');
            qloc=dualsol(qloc);
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
        Qrob(i,:)=[braccio_angles(Q_tot(i,[1 2 3 4 5])) Q_tot(i,6)];
        if any(Qrob(i,:)<ang_pos_rob_min) || any(Qrob(i,:)>ang_pos_rob_max)
            errorflag=1;
        end
    end
    
    for i=1:length(controlvec(:,2))
        if controlvec(i,2)<=0
            errorflag=1;
        end
    end
    
    %jointpos=plot_config([ones(npoints+npoints_first,1) Q_tot]);
    jointpos=plot_config_rob(Qrob);


    print_for_arduino(Qrob,5);
    
    disp(eulr2)
    maxexcurtion=max(Qrob);
    
end
    
end

