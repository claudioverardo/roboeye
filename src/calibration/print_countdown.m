function print_countdown(length)
% PRINT_COUNTDOWN plot on the screen the countdown of length seconds.
%
%   PRINT_COUNTDOWN(length)
%
%   Input arguments:
%   ------------------
%   length: duration of the countdown [s]
%
%   See also PAUSE
    
    for i=1:length
        fprintf(' %d', length-i+1);
        pause(1);
    end
    fprintf('\n');

end