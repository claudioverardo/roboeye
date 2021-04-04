function [sing_flag, sing_vec, Q] = check_sing(start,target)
    %Check if singular config is reached fron servomovment trajectory
    
    sing_flag=false;
    
    Q = braccio_servo_mat(start,target);
    
    sing_vec=zeros(length(Q(:,1)),1);
    
    for i=1:length(Q(:,1))
        q=braccio_angles_inv(Q(i,:));
        J=jacob_diff_kin(q);
        if rank(J)<5
            sing_vec(i)=1;
            sing_flag=true;
        end
    end

end

