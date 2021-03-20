%% Examples of Aruco 8x8 (img_tests 04--09: tests from webcam)
aruco_markers = cell(0);

aruco_markers{1,1} = [
    0 0 0 0 0 0 0 0
    0 0 0 0 1 0 1 0
    0 0 1 1 0 1 0 0
    0 1 0 0 0 1 0 0
    0 0 1 0 0 1 1 0
    0 1 1 1 0 0 1 0
    0 1 1 0 1 0 0 0
    0 0 0 0 0 0 0 0
];
    
aruco_markers{2,1} = [
    0 0 0 0 0 0 0 0
    0 0 0 1 0 0 1 0
    0 0 1 1 1 1 1 0
    0 0 1 0 0 0 0 0
    0 0 0 1 1 1 0 0
    0 0 1 0 0 0 0 0
    0 1 0 0 1 0 1 0
    0 0 0 0 0 0 0 0
];
    
aruco_markers{3,1} = [
    0 0 0 0 0 0 0 0
    0 0 1 0 0 0 1 0
    0 0 0 1 0 1 1 0
    0 1 0 1 0 0 1 0
    0 1 1 0 0 0 0 0
    0 1 0 1 1 0 1 0
    0 1 0 0 1 1 1 0
    0 0 0 0 0 0 0 0
];

aruco_markers{4,1} = [
    0 0 0 0 0 0 0 0
    0 0 1 0 0 0 0 0
    0 1 0 0 1 1 0 0
    0 1 0 0 0 1 0 0
    0 1 1 0 1 0 0 0
    0 0 0 0 1 1 1 0
    0 1 1 0 1 0 1 0
    0 0 0 0 0 0 0 0
];
save('../assets/aruco_markers/aruco_markers_8x8_camera', 'aruco_markers');

%% Examples of Aruco 7x7 (img_tests 01--03)
aruco_markers = cell(0);

aruco_markers{1,1} = [
    0 0 0 0 0 0 0
    0 0 1 1 1 0 0
    0 0 0 1 1 0 0
    0 0 0 1 1 0 0
    0 0 1 0 0 0 0
    0 1 0 1 1 1 0
    0 0 0 0 0 0 0
];
    
aruco_markers{2,1} = [
    0 0 0 0 0 0 0
    0 0 1 1 0 0 0
    0 0 0 1 1 0 0
    0 1 0 0 0 0 0
    0 0 1 0 0 0 0
    0 1 1 1 1 1 0
    0 0 0 0 0 0 0
];
save('../assets/aruco_markers/aruco_markers_7x7', 'aruco_markers');

%% Example of Aruco 8x8 (img_tests iPhone)
aruco_markers = cell(0);

aruco_markers{1,1} = [
    0 0 0 0 0 0 0 0
    0 0 1 0 0 0 1 0
    0 0 0 1 0 1 1 0
    0 1 0 1 0 0 1 0
    0 1 1 0 0 0 0 0
    0 1 0 1 1 0 1 0
    0 1 0 0 1 1 1 0
    0 0 0 0 0 0 0 0
];
save('../assets/aruco_markers/aruco_markers_8x8_iPhone', 'aruco_markers');

%% Examples of Aruco 8x8 (img_test from web)
aruco_markers = cell(0);

aruco_markers{1,1} = [
    0 0 0 0 0 0 0 0
    0 0 1 1 1 1 0 0
    0 1 0 0 0 1 0 0
    0 1 0 0 0 0 1 0
    0 0 1 1 0 1 0 0
    0 0 1 1 0 1 1 0
    0 1 0 1 0 1 1 0
    0 0 0 0 0 0 0 0
];
    
aruco_markers{2,1} = [
    0 0 0 0 0 0 0 0
    0 0 0 0 0 1 1 0
    0 0 0 0 0 1 1 0
    0 1 1 1 1 0 0 0
    0 1 0 0 0 1 0 0
    0 0 1 0 1 0 1 0
    0 0 1 1 0 0 1 0
    0 0 0 0 0 0 0 0
];
    
aruco_markers{3,1} = [
    0 0 0 0 0 0 0 0
    0 1 1 0 0 1 1 0
    0 0 1 1 0 0 0 0
    0 1 0 1 1 1 0 0
    0 1 0 1 0 0 0 0
    0 1 0 0 1 0 0 0
    0 0 0 0 1 1 0 0
    0 0 0 0 0 0 0 0
];
save('../assets/aruco_markers/aruco_markers_8x8_web', 'aruco_markers');