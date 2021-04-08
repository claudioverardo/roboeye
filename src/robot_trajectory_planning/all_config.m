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
        [qloc, fval, info] = inverse_kin_super_simple(transl,q4,startingpos,braccio_params);
        qloc=mod(qloc+180,360)-180;

        %corrections
        corr=z_correction(qloc,transl);
        qloc=qloc+corr;

        %check if solver failed
        if info<=0
            errorflag=true;
        end

        %check for dual solution
        if qloc(3)*qloc(2)<=0
           qloc=dualsol(qloc);
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

