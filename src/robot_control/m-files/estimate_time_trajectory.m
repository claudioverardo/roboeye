function time = estimate_time_trajectory(type_trajectory, trajectory, delta_t)
    
    if strcmp(type_trajectory, 'keypoints') == 1
        
        steps = diff(trajectory);
        max_steps = max(abs(steps),[],2);
        time = round( sum(max_steps)*delta_t ./ 1000 + 0.2 );
        
    elseif strcmp(type_trajectory, 'custom') == 1
        
        time = round( size(trajectory,1)*delta_t ./ 1000 + 0.2 );
        
    end
    
end