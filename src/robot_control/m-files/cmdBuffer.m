classdef cmdBuffer < handle
% CMDBUFFER Create a LIFO buffer to automatically provide input commands.

    properties
        % Cell array of input commands.
        buffer
    end
    
    methods
        
        function obj = cmdBuffer(initial_buffer)
        % Constructor that set the initial buffer.
            if nargin > 0
                obj.buffer = initial_buffer;
            else
                obj.buffer = cell(0);
            end
        end
        
        function cmd = getCmd(obj, str)
        % Get the last command and remove it from the buffer.
        % The interface is the same of input(...)
            if numel(obj.buffer) ~= 0 
                cmd = obj.buffer{1};
                obj.buffer(1) = [];
            else
                cmd = 0;
            end
            fprintf('%s%s\n', str, cmd2str(obj, cmd));
        end
        
        function str = cmd2str(obj, cmd)
        % Convert a command to string to be plotted.
            if isscalar(cmd)
                str = int2str(cmd);
            elseif isvector(cmd)
                str = mat2str(cmd);
            else
                str = '';
            end
        end
        
    end
end