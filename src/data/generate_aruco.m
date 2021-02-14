%% Examples of Aruco 7x7
aruco_markers = cell(0);

aruco_markers{1,1} = [ ...
    0 0 0 0 0 0 0
    0 0 1 1 1 0 0
    0 0 0 1 1 0 0
    0 0 0 1 1 0 0
    0 0 1 0 0 0 0
    0 1 0 1 1 1 0
    0 0 0 0 0 0 0
];
    
aruco_markers{2,1} = [ ...
    0 0 0 0 0 0 0
    0 0 1 1 0 0 0
    0 0 0 1 1 0 0
    0 1 0 0 0 0 0
    0 0 1 0 0 0 0
    0 1 1 1 1 1 0
    0 0 0 0 0 0 0
];
save('data/aruco_markers_7x7', 'aruco_markers');


%% Examples of Aruco 8x8
aruco_markers = cell(0);

aruco_markers{1,1} = [ ...
    % 1st marker
    0 0 0 0 0 0 0 0
    0 0 1 0 0 0 1 0
    0 0 0 1 0 1 1 0
    0 1 0 1 0 0 1 0
    0 1 1 0 0 0 0 0
    0 1 0 1 1 0 1 0
    0 1 0 0 1 1 1 0
    0 0 0 0 0 0 0 0
];
save('data/aruco_markers_8x8', 'aruco_markers');


%% Examples of Aruco 8x8, test with image from web
aruco_markers = cell(0);

aruco_markers{1,1} = [ ...
    % 1st marker
    0 0 0 0 0 0 0 0
    0 0 1 1 1 1 0 0
    0 1 0 0 0 1 0 0
    0 1 0 0 0 0 1 0
    0 0 1 1 0 1 0 0
    0 0 1 1 0 1 1 0
    0 1 0 1 0 1 1 0
    0 0 0 0 0 0 0 0
];
    
aruco_markers{2,1} = [ ...
    % 2nd marker
    0 0 0 0 0 0 0 0
    0 0 0 0 0 1 1 0
    0 0 0 0 0 1 1 0
    0 1 1 1 1 0 0 0
    0 1 0 0 0 1 0 0
    0 0 1 0 1 0 1 0
    0 0 1 1 0 0 1 0
    0 0 0 0 0 0 0 0
];
    
aruco_markers{3,1} = [ ...
    % 3rd marker
    0 0 0 0 0 0 0 0
    0 1 1 0 0 1 1 0
    0 0 1 1 0 0 0 0
    0 1 0 1 1 1 0 0
    0 1 0 1 0 0 0 0
    0 1 0 0 1 0 0 0
    0 0 0 0 1 1 0 0
    0 0 0 0 0 0 0 0
];
save('data/aruco_markers_8x8_web', 'aruco_markers');