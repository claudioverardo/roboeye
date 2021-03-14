function T = external_orientation_camera(P) 
    num_camera = size(P, 2);
    T = cell(1, num_camera);

    for i = 1:num_camera
        [K, R, t] = krt(P{i});
        T{i} = [R t; [0, 0, 0, 1]];
    end
end