%% Examples of Aruco 7x7
aruco_markers = cell(0);

aruco_markers{1,1} = [
    0 0 0 0 0 0 0
    0 0 1 0 1 0 0
    0 1 1 1 1 1 0
    0 1 1 1 1 1 0
    0 1 0 1 0 1 0
    0 0 1 0 1 0 0
    0 0 0 0 0 0 0
];

aruco_markers{2,1} = [
    0 0 0 0 0 0 0
    0 1 0 0 0 0 0
    0 1 0 1 1 1 0
    0 0 1 0 0 1 0
    0 1 0 1 1 1 0
    0 1 0 0 0 0 0
    0 0 0 0 0 0 0
];

aruco_markers{3,1} = [
    0 0 0 0 0 0 0
    0 0 1 1 1 0 0
    0 0 1 0 0 1 0
    0 0 1 1 1 0 0
    0 1 0 1 1 1 0
    0 0 1 0 0 1 0
    0 0 0 0 0 0 0
];

aruco_markers{4,1} = [
    0 0 0 0 0 0 0
    0 1 0 1 0 0 0
    0 1 0 0 1 0 0
    0 1 0 0 1 0 0
    0 0 0 1 1 0 0
    0 1 1 0 0 1 0
    0 0 0 0 0 0 0
];

aruco_markers{5,1} = [
    0 0 0 0 0 0 0
    0 0 0 1 0 0 0
    0 1 0 1 0 1 0
    0 1 0 1 0 1 0
    0 1 0 0 0 1 0
    0 0 1 1 1 0 0
    0 0 0 0 0 0 0
];

aruco_markers{6,1} = [
    0 0 0 0 0 0 0
    0 1 0 0 1 0 0
    0 1 0 0 1 0 0
    0 1 0 0 1 0 0
    0 0 0 0 0 0 0
    0 1 1 1 1 1 0
    0 0 0 0 0 0 0
];

aruco_markers{7,1} = [
    0 0 0 0 0 0 0
    0 1 1 0 0 1 0
    0 1 1 1 0 1 0
    0 1 1 1 0 1 0
    0 0 0 1 0 0 0
    0 1 1 0 1 1 0
    0 0 0 0 0 0 0
];

aruco_markers{8,1} = [
    0 0 0 0 0 0 0
    0 0 1 1 1 0 0
    0 0 0 0 1 0 0
    0 0 0 0 1 0 0
    0 0 1 1 0 0 0
    0 1 0 0 1 1 0
    0 0 0 0 0 0 0
];

aruco_real_sides = [3 3 3 3 4 4 5 5]; % [cm]

% save('../assets/aruco_markers/aruco_markers_7x7', 'aruco_markers');
% save('../assets/aruco_markers/aruco_markers_7x7_real_sides', 'aruco_real_sides');
plot_aruco_markers(aruco_markers);

%% Examples of Aruco 8x8
aruco_markers = cell(0);

aruco_markers{1,1} = [
    0 0 0 0 0 0 0 0
    0 0 1 0 0 1 0 0
    0 1 1 1 0 0 1 0
    0 0 1 0 0 0 1 0
    0 1 0 0 1 0 0 0
    0 1 1 1 0 0 1 0
    0 0 0 1 0 1 1 0
    0 0 0 0 0 0 0 0
];

aruco_markers{2,1} = [
    0 0 0 0 0 0 0 0
    0 1 1 1 1 1 0 0
    0 1 0 0 0 1 0 0
    0 0 0 1 0 1 1 0
    0 1 1 1 0 0 1 0
    0 0 1 1 0 1 1 0
    0 1 0 0 0 1 1 0
    0 0 0 0 0 0 0 0
];

aruco_markers{3,1} = [
    0 0 0 0 0 0 0 0
    0 0 1 1 1 0 0 0
    0 0 1 0 1 0 0 0
    0 1 0 0 0 1 1 0
    0 1 0 1 1 1 0 0
    0 0 0 1 0 1 1 0
    0 0 0 1 0 1 0 0
    0 0 0 0 0 0 0 0
];

aruco_markers{4,1} = [
    0 0 0 0 0 0 0 0
    0 1 0 1 0 0 0 0
    0 0 0 1 1 0 1 0
    0 1 1 0 1 0 0 0
    0 1 0 0 1 1 1 0
    0 0 0 1 1 0 0 0
    0 0 0 1 0 1 1 0
    0 0 0 0 0 0 0 0
];

aruco_markers{5,1} = [
    0 0 0 0 0 0 0 0
    0 1 1 1 0 0 1 0
    0 0 0 0 0 1 1 0
    0 1 1 0 0 1 0 0
    0 1 1 0 1 1 0 0
    0 0 1 0 1 0 0 0
    0 0 1 1 0 1 1 0
    0 0 0 0 0 0 0 0
];

aruco_markers{6,1} = [
    0 0 0 0 0 0 0 0
    0 0 1 0 0 0 1 0
    0 0 0 1 0 1 1 0
    0 1 0 1 0 0 1 0
    0 1 1 0 0 0 0 0
    0 1 0 1 1 0 1 0
    0 1 0 0 1 1 1 0
    0 0 0 0 0 0 0 0
];

aruco_real_sides = [3 3 3 4 4 6]; % [cm]

% save('../assets/aruco_markers/aruco_markers_8x8', 'aruco_markers');
% save('../assets/aruco_markers/aruco_markers_8x8_real_sides', 'aruco_real_sides');
plot_aruco_markers(aruco_markers);

%% Aruco 8x8 for report
aruco_markers = cell(0);

aruco_markers{1,1} = [ % 40
    0 0 0 0 0 0 0 0
    0 0 1 1 0 0 1 0
    0 0 0 0 0 0 0 0
    0 1 0 0 1 1 1 0
    0 1 0 1 0 0 0 0
    0 1 0 1 0 0 0 0
    0 0 0 1 0 1 1 0    
    0 0 0 0 0 0 0 0
];

aruco_markers{2,1} = [ % 41
    0 0 0 0 0 0 0 0
    0 0 1 1 0 0 0 0
    0 0 0 0 1 0 1 0
    0 0 0 1 1 0 1 0
    0 1 1 1 0 1 0 0
    0 1 0 0 0 1 0 0
    0 0 1 0 0 0 1 0
    0 0 0 0 0 0 0 0
];

aruco_markers{3,1} = [ % 42
    0 0 0 0 0 0 0 0
    0 0 1 1 0 0 0 0
    0 0 1 0 1 0 1 0
    0 1 0 0 1 0 0 0
    0 0 0 0 1 1 0 0
    0 1 0 0 1 1 0 0
    0 1 1 1 0 1 0 0
    0 0 0 0 0 0 0 0
];

aruco_markers{4,1} = [ % 43
    0 0 0 0 0 0 0 0
    0 0 1 1 0 1 0 0
    0 1 1 1 1 1 1 0
    0 1 1 1 1 0 1 0
    0 1 1 1 0 0 0 0
    0 1 1 0 1 0 1 0
    0 1 1 1 0 1 1 0
    0 0 0 0 0 0 0 0
];

aruco_markers{5,1} = [ % 44
    0 0 0 0 0 0 0 0
    0 0 1 1 1 0 0 0
    0 0 0 1 0 1 0 0
    0 1 1 0 1 1 0 0
    0 0 1 0 1 1 0 0
    0 1 0 1 0 0 1 0
    0 0 0 1 1 1 1 0
    0 0 0 0 0 0 0 0
];

aruco_markers{6,1} = [ % 45
    0 0 0 0 0 0 0 0
    0 0 1 1 1 0 1 0
    0 0 1 1 0 0 0 0
    0 0 1 0 0 0 1 0
    0 1 0 1 1 1 1 0
    0 0 1 1 1 0 0 0
    0 0 1 1 0 1 0 0
    0 0 0 0 0 0 0 0
];

aruco_markers{7,1} = [ % 46
    0 0 0 0 0 0 0 0
    0 0 1 1 1 1 0 0
    0 1 0 1 0 0 1 0
    0 0 1 0 1 0 0 0
    0 0 1 1 0 0 1 0
    0 0 0 1 0 1 1 0
    0 1 1 1 1 0 0 0
    0 0 0 0 0 0 0 0
];

aruco_markers{8,1} = [ % 47
    0 0 0 0 0 0 0 0
    0 1 0 0 0 0 1 0
    0 1 0 0 0 0 0 0
    0 1 0 0 1 0 1 0
    0 1 1 0 1 1 0 0
    0 0 0 0 0 1 0 0
    0 1 0 1 0 1 0 0
    0 0 0 0 0 0 0 0
];

aruco_markers{9,1} = [ % 48
    0 0 0 0 0 0 0 0
    0 1 0 0 0 1 0 0
    0 1 0 0 0 1 0 0
    0 1 1 0 1 0 1 0
    0 0 0 0 1 0 0 0
    0 1 1 0 0 0 0 0
    0 1 1 1 1 1 1 0
    0 0 0 0 0 0 0 0
];

aruco_markers{10,1} = [ % 49
    0 0 0 0 0 0 0 0
    0 1 0 0 1 0 0 0
    0 1 1 1 1 1 0 0
    0 1 0 1 1 0 1 0
    0 1 1 1 0 0 0 0
    0 1 0 1 1 0 0 0
    0 0 1 0 1 0 0 0
    0 0 0 0 0 0 0 0
];

aruco_markers{11,1} = [ % 50
    0 0 0 0 0 0 0 0
    0 1 0 0 1 1 0 0
    0 0 0 1 0 0 0 0
    0 1 1 0 1 1 0 0
    0 1 0 1 0 0 0 0
    0 0 1 0 0 1 1 0
    0 0 1 0 1 0 0 0
    0 0 0 0 0 0 0 0
];

aruco_markers{12,1} = [ % 51
    0 0 0 0 0 0 0 0
    0 1 0 0 1 1 1 0
    0 1 0 1 1 0 1 0
    0 1 1 1 0 0 0 0
    0 1 0 1 0 1 1 0
    0 0 0 1 1 1 1 0
    0 0 0 1 0 0 0 0
    0 0 0 0 0 0 0 0
];

aruco_markers{13,1} = [ % 52
    0 0 0 0 0 0 0 0
    0 1 0 1 0 0 1 0
    0 0 1 0 0 1 0 0
    0 1 0 0 1 1 1 0
    0 1 0 0 0 0 0 0
    0 0 1 1 1 1 0 0
    0 1 1 1 0 0 0 0
    0 0 0 0 0 0 0 0
];

aruco_markers{14,1} = [ % 53
    0 0 0 0 0 0 0 0
    0 1 0 1 1 0 1 0
    0 0 1 1 0 0 1 0
    0 0 0 1 1 1 0 0
    0 1 1 1 0 0 0 0
    0 0 1 0 1 0 1 0
    0 0 1 1 1 1 1 0
    0 0 0 0 0 0 0 0
];

aruco_markers{15,1} = [ % 54
    0 0 0 0 0 0 0 0
    0 1 0 1 1 0 1 0
    0 1 1 1 1 1 1 0
    0 1 0 0 0 1 1 0
    0 1 0 0 1 0 0 0
    0 0 0 1 0 0 1 0
    0 1 0 1 1 1 1 0
    0 0 0 0 0 0 0 0
];

aruco_markers{16,1} = [ % 60
    0 0 0 0 0 0 0 0
    0 1 1 0 0 1 1 0
    0 0 1 0 0 0 0 0
    0 1 1 0 0 1 0 0
    0 1 0 0 1 1 0 0 
    0 0 0 1 0 0 1 0
    0 1 1 0 0 1 0 0
    0 0 0 0 0 0 0 0
];

aruco_markers{17,1} = [ % 61
    0 0 0 0 0 0 0 0
    0 1 1 0 0 1 0 0
    0 0 1 0 1 0 0 0 
    0 0 0 1 1 0 1 0
    0 0 1 1 1 0 1 0
    0 0 1 0 0 0 1 0
    0 0 0 1 1 0 1 0
    0 0 0 0 0 0 0 0
];

aruco_markers{18,1} = [ % 62
    0 0 0 0 0 0 0 0
    0 1 1 0 0 1 1 0
    0 1 1 1 0 1 1 0
    0 1 1 1 0 1 0 0
    0 0 0 0 0 0 0 0
    0 1 1 1 1 0 0 0
    0 1 1 0 1 0 0 0
    0 0 0 0 0 0 0 0
];

aruco_markers{19,1} = [ % 63
    0 0 0 0 0 0 0 0
    0 1 1 1 0 0 1 0
    0 0 1 0 1 1 1 0
    0 1 1 0 1 0 0 0
    0 0 1 0 1 0 1 0
    0 1 0 0 0 0 1 0
    0 1 1 0 1 1 1 0
    0 0 0 0 0 0 0 0
];

aruco_markers{20,1} = [ % 64
    0 0 0 0 0 0 0 0
    0 1 1 1 0 1 1 0
    0 1 1 1 1 0 0 0
    0 0 1 1 0 1 0 0
    0 0 0 0 1 0 1 0
    0 1 0 0 0 1 1 0
    0 1 0 1 0 0 1 0
    0 0 0 0 0 0 0 0
];

aruco_real_sides = [3 3 3 3 3 3 3 3 3 3 3.5 3.5 3.5 3.5 3.5 4 4 4 4 4]; % [cm]

% save('../assets/aruco_markers/aruco_markers_vision_testbench', 'aruco_markers');
% save('../assets/aruco_markers/aruco_markers_vision_testbench_real_sides', 'aruco_real_sides');
plot_aruco_markers(aruco_markers);
