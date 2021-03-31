classdef cmdBuffer < handle
    properties
        buffer
    end
    methods
        function obj = cmdBuffer(initial_buffer)
            if nargin > 0
                obj.buffer = initial_buffer(:);
            else
                obj.buffer = [];
            end
        end
        function cmd = getCmd(obj, str)
            if numel(obj.buffer) ~= 0 
                cmd = obj.buffer(1);
                obj.buffer(1) = [];
            else
                cmd = 0;
            end
            fprintf('%s%d\n', str, cmd);
        end
    end
end