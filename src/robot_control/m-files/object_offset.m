function dt = object_offset(dh, dr, t, nz)
% OBJECT_OFFSET Find the offset of the end-effector position to adjust the
% grasping position on the basis of object data (in robot coordinates).
%
%   dt = OBJECT_OFFSET(dh, dr, t, nz)
%
%   Input arguments:
%   ------------------
%   dh: height offset in object frame
%   dr: radial offset in robot frame
%   t:  [x,y,z] is the initial position of the end-effector in robot frame
%   nz: normal versor along which dh is applied (default [0 0 1])
%
%   Output arguments:
%   ------------------
%   dt: [dx,dy,dz] is the position offset of the end-effector in robot frame
%
%   See also GENERATE_TRAJECTORY
    
    if nargin <= 3
        nz = [0 0 1];
    end

    r = norm([t(1) t(2)]);
    
    dx=nz(1)*dh + dr/r*t(1);
    dy=nz(2)*dh + dr/r*t(2);
    dz=nz(3)*dh;
    
    dt = [dx, dy, dz];
    
end

