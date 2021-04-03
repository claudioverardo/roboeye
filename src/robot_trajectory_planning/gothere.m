function [qrob,errorflag,q] = gothere(x,y,z,roll,grasp)
%Returns joint's angular position for a given spacial position in phisical
%space
%   the function decides autonomously the end effector orientation in order
%   to manage to reach the spatial point in the biggest workspace possible

if nargin <= 3
    roll = 90;
    grasp = 73;
end

%initialization flags
errorflag=false;

%define translation
transl=[x y z];


if x>=0 %frontal workspace
    dirindex=-1;
else %backward workspace
    dirindex=1;
end

[qrob,q,foundflag] = scanEFangle(transl,dirindex);

%check if algorithm found a solution and if not check other direction
if foundflag==false 
    dirindex=-dirindex;
    [qrob,q,foundflag] = scanEFangle(transl,dirindex);
end

%check in fo solution was found
if foundflag==false 
    errorflag=true;
    % disp("------no solution found------")
else
    % disp("------solution found------")
end

%jp=plot_config_rob(qrob);

%set grabber angle
qrob(5)=roll;
qrob(6)=grasp;


%jp=plot_config([0 q])
end

