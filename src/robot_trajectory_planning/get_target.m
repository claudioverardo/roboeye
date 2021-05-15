function [target, i_aruco] = get_target(method, QNUM, cam, vision_args, fn_robot_input)
% GET_TARGET Retrieve the position of a target in the scene (world frame).
%
%   [target, i_aruco] = GET_TARGET(method, QNUM, cam, vision_args, fn_robot_input)
%
%   Input arguments:
%   ------------------
%   method: type of target acquisition
%       - 'q': position in joint space from user
%       - '3d-npoints': n positions in 3d space from user (world frame)
%       - '3d-vision': position in 3d space from user or camera (world frame)
%   QNUM: number of joints of the robot
%   cam: webcam object of the camera, cf. webcam(...)
%   vision_args: struct of vision parameters
%   fn_robot_input: function to acquire input, cf. input(...) or cmdBuffer
%
%   Output arguments:
%   ------------------
%   target: position ot the chosen target (world frame)
%   i_aruco: id of the marker associated to the target (0 if none)
%
%   NOTE: this function requires the MATLAB Support Package for USB Webcams.
%   For details regarding vision_args refers to get_target_from_vision(...).
%
%   See also GENERATE_TRAJECTORY, GET_TARGET_FROM_VISION

    invalid_target = 1;
    while invalid_target
        
        switch method
            
            case 'q'
                target = cmd_acquire( ...
                    '', ...
                    @(x) isvector(x) && ( ( numel(x)==QNUM && check_limits_joints(x) ) || numel(x)==3 ), ...
                    fn_robot_input, ...
                    '   Insert position [M1,M2,M3,M4,M5,M6]: ', ...
                    '   Position not valid\n' ...
                );
                i_aruco = 0;
                invalid_target = 0;
            
            case '3d-npoints'
            
                n_points = cmd_acquire( ...
                    '', ...
                    @(x) isscalar(x) && isnumeric(x) && x>0, ...
                    fn_robot_input, ...
                    'Number of points: ', ...
                    'Number not valid\n' ...
                );

                target = zeros(n_points,3);        
                for i=1:n_points
                    target(i,:) = cmd_acquire( ...
                        '', ...
                        @(x) isvector(x) && numel(x)==3, ...
                        fn_robot_input, ...
                        sprintf('   Point %d [X Y Z]: ', i), ...
                        '   Point not valid\n' ...
                    );
                end
                i_aruco = zeros(1,n_points);
                invalid_target = 0;
            
            case '3d-vision'
                cmd = cmd_acquire( ...
                    'Available commands:\n   1: target from camera\n   2: target from user\n', ...
                    @(x) isscalar(x) && ( x == 1 || x == 2 ), ...
                    fn_robot_input ...
                );

                switch cmd
                    case 1
                        [t, R, i_aruco] = get_target_from_vision(cam, vision_args, fn_robot_input);
                        target = t;
                    case 2
                        target = cmd_acquire( ...
                            '   Insert target position\n', ...
                            @(x) isvector(x) && numel(x)==3, ...
                            fn_robot_input, ...
                            '   [X,Y,Z] = ', ...
                            '   Position not valid\n' ...
                        ); 
                        i_aruco = 0;
                end

                if numel(target) == 0
                    invalid_target = 1; % when vision does not find anything
                else 
                    invalid_target = 0;
                end
                
        end

    end

end