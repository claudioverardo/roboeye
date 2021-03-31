function [qrob,errorflag,q] = gothere(x,y,z)
%Returns joint's angular position for a given spacial position in phisical
%space
%   the function decides autonomously the end effector orientation in order
%   to manage to reach the spatial point in the biggest workspace possible

% define robot agles limits
ang_pos_rob_max=[180 165 180 180 180 73];
ang_pos_rob_min=[0 15 0 0 0 0];

%initialization flags
errorflag=false;

%define translation
transl=[x y z];


if x>=0 %frontal workspace
    dirindex=-1;
else %backward workspace
    dirindex=1;
end

[qrob,q,foundflag] = scanEFangle(transl,dirindex,ang_pos_rob_min,ang_pos_rob_max);

%check if algorithm found a solution and if not check other direction
if foundflag==false 
    dirindex=-dirindex;
    [qrob,q,foundflag] = scanEFangle(transl,dirindex,ang_pos_rob_min,ang_pos_rob_max);
end

%check in fo solution was found
if foundflag==false 
    errorflag=true;
    disp("------no solution found------")
else
    disp("------solution found------")
end

jp=plot_config_rob(qrob)
%jp=plot_config([0 q])
end

