classdef cmdBuffer < handle
    properties
        buffer
    end
    
    methods
        
        function obj = cmdBuffer(initial_buffer)
            if nargin > 0
                obj.buffer = initial_buffer;
            else
                obj.buffer = cell(0);
            end
        end
        
        function cmd = getCmd(obj, str)
            if numel(obj.buffer) ~= 0 
                cmd = obj.buffer{1};
                obj.buffer(1) = [];
            else
                cmd = 0;
            end
            fprintf('%s%s\n', str, cmd2str(obj, cmd));
        end
        
        function str = cmd2str(obj, cmd)
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