function Q = braccio_servo_mat(start,target)
    %Function that emulates braccio.servomovment
    
    target=round(target);
    qloc=round(start);
    pred_size=max(abs(qloc-target));
    clear Q;
    Q=zeros(pred_size,5);
    
    i=1;

    while any(qloc ~= target)
        Q(i,:)=qloc; 
        i=i+1;    
        qloc=qloc+sign(target-qloc);
    end
    
    Q(i,:)=qloc; 
end



