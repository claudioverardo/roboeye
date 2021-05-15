function Jpos = joint_pos(q,braccio_params)
% JACOB_DIFF_KIN Compute the spatial position of the joints in workspace.
%.
%   Jpos = joint_pos(q,braccio_params)
%
%   Input arguments:
%   ------------------
%   q:                  1xQNUM-1 array, joints positions in model convention
%   braccio_params:     1xQNUM-1 array, real parameters of the Braccio robot,
%                       cf. direct_kin(...)
%
%   Output arguments:
%   ------------------
%   Jpos:               QNUM-1x3 x,y,and z foint position in workspace
%                       (robot convention)
    
    Jpos = zeros(length(q),3);
    Qcorr=q+[0 90 0 -90 0]; %convert angles to DH convenction

    for i=1:5
        Afin=direct_kin(Qcorr,i,braccio_params);
        Jpos(i+1,:)=Afin([1 2 3],4)';
    end
end