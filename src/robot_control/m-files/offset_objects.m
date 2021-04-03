function dt = offset_objects(dh,dr,t,nz)
    %Find offset in Robot coordinates using object data and position

    % dh is the height's offset in the object rf
    % dr is the radial offset in ROBOT rf (not in object rf because the object must be graspable)
    % t = [x,y,z] are the arucos's center robot coordinates
    % nz is the aruco's normal vector in robot coords
    % dt = dx,dy,dz are the offset in robot coordinates

    r = norm([t(1) t(2)]);
    
    dx=nz(1)*dh + dr/r*t(1);
    dy=nz(2)*dh + dr/r*t(2);
    dz=nz(3)*dh;
    
    dt = [dx, dy, dz];
end

