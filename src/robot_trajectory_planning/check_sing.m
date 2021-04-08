function [sing_flag, sing_vec] = check_sing(Q, home)
% CHECK_SING Check if there are singular configuration among an input set of
% points in the space of joints.
%
%   [sing_flag, sing_vec] = check_sing(Q, home)
%
%   Input arguments:
%   ------------------
%   Q:          NxQNUM array, set of points under test (arranged by rows)
%   home:       1xQNUM array, home position of the robot (avoid extra warnings)
%   
%   Output arguments:
%   ------------------
%   sing_flag:  1 if at least one singularity is found, 0 otherwise
%   sing_vec:   sing_vec(i) = 1 if Q(i,:) is singular, 0 otherwise
%
% See also JACOB_DIFF_KIN

    home = round(home);
    sing_vec=zeros(size(Q,1),1);
    
    for i=1:size(Q,1)
        
        % home_position is singular?
        % yes, but actually no...
        if ~all(round(Q(i,:)) == home)
            
            % Check if the given joints position is singular
            q=braccio_angles_inv(Q(i,1:end-1),[],home(1:end-1));
            J=jacob_diff_kin(q);
            if rank(J)<5
                sing_vec(i)=1;
            end
            
        end
        
    end
    
    sing_flag = any(sing_vec);

end

