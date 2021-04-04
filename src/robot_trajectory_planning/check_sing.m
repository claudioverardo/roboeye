function [sing_flag, sing_vec, Q] = check_sing(start,target)
    %Check if singular config is reached fron servomovment trajectory
    
    sing_flag=0;
    
    Q = braccio_servo_mat(start,target);
    
    sing_vec=zeros(length(Q(:,1)),1);
    
    for i=1:length(Q(:,1))
        J=jacob_diff_kin(Q(i,:));
        if rank(J)<5
            sing_vec(i)=1;
            sing_flag=1;
        end
    end

end

