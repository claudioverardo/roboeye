function J = jacob_diff_kin(q, braccio_params, prism)
% JACOB_DIFF_KIN Compute the geometric Jacobian of the Braccio robot for a
% given position of the joints.
%.
%   J = JACOB_DIFF_KIN(q, braccio_params, prism)
%
%   Input arguments:
%   ------------------
%   q:                  1xQNUM-1 array, joints positions in model convention
%   braccio_params:     1xQNUM-1 array, real parameters of the Braccio robot,
%                       cf. direct_kin(...)
%   prism:              1xnjoins logical array, true if the i-th joint is
%                       prismatic, false if it is rotoidal
%
%   Output arguments:
%   ------------------
%   J:                  6xQNUM-1 geometric Jacobian matrix of the robot
%
% See also CHECK_SING
    
    % define number of joints
    njoints=5;
    
    % if nargin <=2
    %     braccio_params=[71 125 125 195 0];
    % end
    
    if nargin <=3
        prism=zeros(njoints,1);
    end
    

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

