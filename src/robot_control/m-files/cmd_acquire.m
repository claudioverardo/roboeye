function cmd = cmd_acquire(help, fn_val, fn_robot_input, cmd_ask_str, cmd_not_valid_str)
% CMD_ACQUIRE Acquire a command that satisfy a given validation function.
% Both manual input from user and automatic input from buffer are supported.
%
%   cmd = CMD_ACQUIRE(help, fn_val, fn_robot_input, cmd_ask_str, cmd_not_valid_str)
%
%   Input arguments:
%   ------------------
%   help:               help message to be displayed before acquisition
%   fn_val:             validation function of the command
%   fn_robot_input:     function to acquire input, cf. input(...) or cmdBuffer
%   cmd_ask_str:        message to require a command (optional)
%   cmd_not_valid_str:  message if the acquired command is invalid (optional)
%
%   Output arguments:
%   ------------------
%   cmd:                command acquired
%
%   See also CMD_EXECUTE

    if nargin <= 3
        cmd_ask_str = 'Command: ';
        cmd_not_valid_str = 'Command not valid\n';
    end
   
    fprintf(help);
    
    check_cmd_sw = 0;
    while ~check_cmd_sw
        cmd = fn_robot_input(cmd_ask_str);
        check_cmd_sw = fn_val(cmd);
        if ~check_cmd_sw
            fprintf(cmd_not_valid_str);
        end
    end  
    
end