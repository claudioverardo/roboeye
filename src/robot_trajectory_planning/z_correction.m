function corr=z_correction(in,transl)
    %Implement z compensation 

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
end
