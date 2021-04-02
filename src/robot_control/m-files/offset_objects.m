function [dx,dy,dz] = offset_objects(dh,dr,x,y,z,nz)
    %Find offset in Robot coordinates using object data and position

    % dx,dy,dz are the offset in robot coordinates
    % x,y,z are arucos's center robot coordinates
    % nz is the aruco's normal vector in robot coords
    % dh is the height's offset in the object rf
    % dr is the radial offset in ROBOT rf (not in object rf because the object must be graspable)

    dx=nz(1)*dh + dr/r*x;
    dy=nz(2)*dh + dr/r*y;
    dz=nx(3)*dh;
end

