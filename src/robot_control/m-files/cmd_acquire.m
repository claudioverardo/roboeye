function cmd = cmd_acquire(help, fn_val, fn_robot_input, cmd_ask_str, cmd_not_valid_str)

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