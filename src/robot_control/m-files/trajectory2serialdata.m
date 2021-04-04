function data_tx = trajectory2serialdata(trajectory_type, delta_T, trajectory)

    switch trajectory_type
        case 'custom'
            trajectory_type_id = 1;
        case 'keypoints'
            trajectory_type_id = 2;
        otherwise
            trajectory_type_id = 0;
    end
    
    n_points_trajectory = size(trajectory,1);
    trajectory_vec = reshape(trajectory',1,[]);
    
    data_tx = uint8( [trajectory_type_id n_points_trajectory delta_T trajectory_vec] );

end