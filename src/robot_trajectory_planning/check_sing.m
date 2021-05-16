function [sing_flag, sing_vec] = check_sing(Q, braccio_params)
% CHECK_SING Check if there are singular configuration among a given set of
% points in the space of joints (in model convention).
%
%   [sing_flag, sing_vec] = CHECK_SING(Q, braccio_params)
%
%   Input arguments:
%   ------------------
%   Q:                  NxQNUM-1 array, set of joints positions under test
%   braccio_params:     1xQNUM-1 array, real parameters of the Braccio robot,
%                       cf. direct_kin(...)
%   
%   Output arguments:
%   ------------------
%   sing_flag:  1 if at least one singularity is found, 0 otherwise
%   sing_vec:   sing_vec(i) = 1 if Q(i,:) is singular, 0 otherwise
%
% See also JACOB_DIFF_KIN

    sing_vec=zeros(size(Q,1),1);
    
    for i=1:size(Q,1)
            
        % Check if the given joints position is singular
        J=jacob_diff_kin(Q(i,:), braccio_params);
        if rank(J)<5
            sing_vec(i)=1;
        end
        
    end
    
    sing_flag = any(sing_vec);

end

