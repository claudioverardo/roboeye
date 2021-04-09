function corr = z_correction(qloc, transl)
% Z_CORRECTION Manual tuning of the joints positions in order to fix the z
% positions reached by the end effector.
%
%   corr = Z_CORRECTION(in, transl)
%
%   Input arguments:
%   ------------------
%   qloc:       1xQNUM array, angular positions of the joints
%   transl:     1x3 array, translation vector of the end effector
%
%   Output arguments:
%   ------------------
%   corr:       1xQNUM array, corrected joints position

    z=transl(3)+5;
    
    corr2=0;
    corr3=0;
    corr4=0;
    
    % if z>=0 && z<50
    %     corr2=7*z/50;
    % elseif z>=50 && z<100
    %     corr2=2*(z-50)/50+7;
    % elseif z>=100 && z<150
    %     corr2=1*(z-100)/50+9;
    % elseif z>=150 && z<200
    %     corr2=-1*(z-150)/50+10;
    %     corr3=1*(z-150)/50;
    % elseif z>=200 && z<250
    %     corr2=-9*(z-200)/50+9;
    %     corr3=-1*(z-200)/50+1;
    % else
    %     corr2=0;
    % end

    %%% UPDATE 07/04/2021
%     if z>=0 && z<50
%         corr2=4*z/50-4;
%     elseif z>=50 && z<100
%         corr2=4*(z-50)/50-1;
%     elseif z>=100 && z<150
%         corr2=2*(z-100)/50+5-2;
%     elseif z>=150 && z<200
%         corr2=1*(z-150)/50+7-2;
%     elseif z>=200 && z<250
%         corr2=0*(z-200)/50+8-1;
%     elseif z>=250 && z<300
%         corr2=-8*(z-250)+8-1;
%     else
%         corr2=0;
%     end

    %%% UPDATE 09/04/2021
    if z>=0 && z<50
        corr2=2*z/50;
        corr4=0*z/50+4;
    elseif z>=50 && z<100
        corr2=1*(z-50)/50+2;
        corr4=0*z/50+4;
    elseif z>=100 && z<150
        corr2=4*(z-100)/50+3;
        corr4=0*z/50+4;
    elseif z>=150 && z<200
        corr2=0*(z-150)/50+7;
        corr4=0*z/50+4;
    elseif z>=200 && z<250
        corr2=1*(z-200)/50+7;
        corr4=0*z/50+4;
    elseif z>=250 && z<300
        corr2=-8*(z-250)+8;
        corr4=-4*z/50+4;
    else
        corr2=0;
    end
    
    corr=[0 corr2 corr3 -corr4 0];%*transl(3)/50;
    %corr=[0 0 0 0 0];
    
end

