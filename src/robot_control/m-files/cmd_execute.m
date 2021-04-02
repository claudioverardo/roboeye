function cmd_err = cmd_execute(s, cmd, data_tx, cmd_ack_str, cmd_no_ack_str)
% CMD_EXECUTE TODO
%
%   cmd_err = CMD_EXECUTE(s, cmd, data_tx, cmd_ack_str, cmd_no_ack_str)
%
%   Input arguments:
%   ------------------
%   s:
%   cmd:
%   data_tx: 
%   cmd_ack_str:
%   cmd_no_ack_str:
%
%   Output arguments:
%   ------------------
%   cmd_err:   
%
%   See also TODO

    if nargin <= 3
        cmd_ack_str = 'ACK received\n';
        cmd_nack_str = 'No ACK from the robot!!!\n';
    end
    
    write(s, uint8(cmd), 'uint8');
    cmd_robot = uint8(read(s,1,'uint8'));
    check_cmd_hw = cmd_robot == cmd;
    if ~check_cmd_hw
        fprintf(cmd_nack_str);
        cmd_err = 1;
    else
        fprintf(cmd_ack_str);
        cmd_err = 0;
        if nargin > 2 && ~isempty(data_tx)
            write(s, data_tx, 'uint8');
        end
    end
                        
end