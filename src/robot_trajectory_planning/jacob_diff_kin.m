function J = jacob_diff_kin(q, braccio_params)
% JACOB_DIFF_KIN Function that computes the geometric Jacobian of the robot.
%
%   J = JACOB_DIFF_KIN(q, braccio_params)
%
%   Input arguments:
%   ------------------
%   q:                  angular positions of the joints
%   braccio_params:     1xQNUM-1 array, real distances between robot joints
%
%   Output arguments:
%   ------------------
%   J:                  geometric Jacobian matrix of the robot
%
% See also CHECK_SING
    
    % if nargin <=2
    %     braccio_params=[71 125 125 195 0];
    % end

    njoints=5;
    prism=zeros(njoints,1);

    %initialization
    Jp=zeros(3,njoints);
    Jo=zeros(3,njoints);
    J=zeros(6,njoints);
    z=zeros(njoints,3);
    p=zeros(njoints,3);


    for i=1:njoints
        A=direct_kin(q,i-1,braccio_params);
        R=A([1 2 3],[1 2 3]);
        p(i,:)=A([1 2 3],4);
        z(i,:)=(R*[0 0 1]')';
    end

    A=direct_kin(q,njoints,braccio_params);
    pe=A([1 2 3],4);

    for i=1:njoints
        if prism(i) == true
            Jp(:,i)=z(i,:);
            Jo(:,i)=0;
        else
            Jp(:,i)=cross(z(i,:)',(pe-p(i,:)'));
            Jo(:,i)=z(i,:);
        end
        J(:,i)=[Jp(:,i); Jo(:,i)]';
    end

end

