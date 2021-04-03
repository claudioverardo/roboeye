function [qrob,q_teo,foundflag] = scan_EF_pitch(transl,dirindex,braccio_params)
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
        [qloc, fval, info] = inverse_kin_super_simple(transl,joint4,startingpos,braccio_params);
        qloc=mod(qloc+180,360)-180;
        %q([1 2 3 4 5])=qloc;
        
        %corrections
        z=transl(3)+5;
        corr3=0;
        if z>=0 && z<50
            corr2=7*z/50;
        elseif z>=50 && z<100
            corr2=2*(z-50)/50+7;
        elseif z>=100 && z<150
            corr2=1*(z-100)/50+9;
        elseif z>=150 && z<200
            corr2=-1*(z-150)/50+10;
            corr3=1*(z-150)/50;
        elseif z>=200 && z<250
            corr2=-9*(z-200)/50+9;
            corr3=-1*(z-200)/50+1;
        else
            corr2=0;
        end
        
        corr=[0 corr2 corr3 0 0];%*transl(3)/50;
        qloc=qloc+corr;

        %check for solver errors
        controlvec=[sum(sum(fval.^2)) info];

        %check if solver failed
        if controlvec(2)<=0
            errorflag=true;
        end
        
        %check for dual solution
        if qloc(3)*qloc(2)<=0
           qloc=dualsol(qloc);
        end

        %convert joint pos into robot's rf
        qrob=[braccio_angles(qloc) q(6)];    
        if ~check_limits_joints(qrob) %if the solution is not in the range check for dual
           qloc=dualsol(qloc);
           qrob=[braccio_angles(qloc) q(6)];
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
        % plot_config_rob(qrob)
        % disp(corr2);
    end
end

