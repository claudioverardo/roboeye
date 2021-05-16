%% Dictionary of objects associated to aruco_markers_7x7
% offsets in [mm]
objects_dict = struct();

objects_dict(1).name = 'cup';
objects_dict(1).offset_h = -30;
objects_dict(1).offset_r = 0;
objects_dict(1).offset_ef = -40;

objects_dict(2).name = 'sharpener';
objects_dict(2).offset_h = -5;
objects_dict(2).offset_r = 0;
objects_dict(2).offset_ef = -45;

objects_dict(3).name = 'glass';
objects_dict(3).offset_h = -20;
objects_dict(3).offset_r = 0;
objects_dict(3).offset_ef = -40;

objects_dict(4).name = 'glue';
objects_dict(4).offset_h = -40;
objects_dict(4).offset_r = 0;
objects_dict(4).offset_ef = -40;

objects_dict(5).name = 'candies';
objects_dict(5).offset_h = -15;
objects_dict(5).offset_r = 0;
objects_dict(5).offset_ef = -5;

objects_dict(6).name = 'tissues';
objects_dict(6).offset_h = -10;
objects_dict(6).offset_r = 0; %20
objects_dict(6).offset_ef = -10;

objects_dict(7).name = 'foo';
objects_dict(7).offset_h = 0;
objects_dict(7).offset_r = 0;
objects_dict(7).offset_ef = 0;

objects_dict(8).name = 'foo';
objects_dict(8).offset_h = 0;
objects_dict(8).offset_r = 0;
objects_dict(8).offset_ef = 0;

% save('../assets/objects_dict/objects_dict_7x7', 'objects_dict');
% plot_aruco_markers(load('../assets/aruco_markers/aruco_markers_7x7.mat').aruco_markers);
