function print_countdown(length)
    % length [s]
    
    for i=1:length
        fprintf(' %d', length-i+1);
        pause(1);
    end
    fprintf('\n');

end