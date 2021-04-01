function [qrob,q,foundflag] = scanEFangle(transl,dirindex,ang_pos_rob_min,ang_pos_rob_max)
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
        [qloc, fval, info] = inverse_kin_super_simple(transl,joint4,startingpos);
        qloc=mod(qloc+180,360)-180;
        %q([1 2 3 4 5])=qloc;       

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
        if any(qrob<ang_pos_rob_min) || any(qrob>ang_pos_rob_max) %if the solution is not in the range check for dual
           qloc=dualsol(qloc);
           qrob=[braccio_angles(qloc) q(6)];
           if any(qrob<ang_pos_rob_min) || any(qrob>ang_pos_rob_max) %if even dual is out of range trigger error flag
               errorflag=true;
           end    
        end
        
        % define q in algorithm rf
        q([1 2 3 4 5])=qloc; 

        if errorflag==false
            exitflag=true;
            foundflag=true;
        end
        
        if eulr_index>99
            exitflag=true;
        end
        plot_config_rob(qrob)
    end
end

