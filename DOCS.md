# Documentation
1. [Robot Calibration](#robot-calibration)
2. [Robot Vision](#robot-vision)
3. [Robot Control](#usage)

# Code Overview
+ Robot Calibration: code for the calibration of the robot with the checkerboard
+ Robot Vision: code for the Aruco detection, Aruco pose estimation
+ Robot Control: code for trajectory planning, direct kinematics, inverse kinematics, arduino fsm

<a name="robot-calibration"></a>
## Robot Calibration
<!-- acquire_calibration_images matlab function -->
<details>
    <summary>
        acquire_calibration_images
    </summary>

    images = acquire_calibration_images(n_images, cameras, dirs_images)

Input params:
+ **n_images**: number of the images to be acquired from each camera
+ **cameras**: cell array of camera objects (cf. webcam(...))
+ **dirs_images**: cell array with the directory paths where to save the images

Output params:
+ **images**: cell array of acquired images. images{i,j} is the i-th image acquired from the j-th camera
</details>

<!-- calibration_extrinsics_camera matlab function -->
<details>
    <summary>
        calibration_extrinsics_camera
    </summary>

    [R_cam, t_cam] = calibration_extrinsics_camera(cam, K, step_size, grid_arrangement, cm2px_scale, dir)

Input params:
+ **cam**:                webcam object (cf. webcam(...))
+ **K**:                  intrinsics matrix of the camera (literature convention)
+ **step_size**:          side of the squares of the checkerboard [cm]
+ **grid_arrangement**:   [x-steps y-steps] steps of the checkerboard along x,y axes
+ **cm2px_scale**:        dimension in cm of 1 pixel of the rectified image
+ **dir**:                directory where to write/read the calibration files

Output params:
+ **R_cam**: rotation matrix of the camera extrinsics in the world frame (literature convention)
+ **t_cam**: translation vector of the camera extrinsics in the world frame (literature convention)
</details>

<!-- calibration_extrinsics_stereo matlab function -->
<details>
    <summary>
        calibration_extrinsics_stereo
    </summary>

    [delta_R, delta_t, E, F] = calibration_extrinsics_stereo(K1, R1, t1, K2, R2, t2, dir)

Input params:
+ **K1**:         intrinsics matrix of the first camera (literature convention)
+ **R1**:         rotation matrix of the extrinsics of the first camera in the world frame (literature convention)
+ **t1**:         translation vector of the extrinsics of the first camera in the world frame (literature convention)
+ **K2**:         intrinsics matrix of the second camera (literature convention)
+ **R2**:         rotation matrix of the extrinsics of the second camera in the world frame (literature convention)
+ **t2**:         translation vector of the extrinsics of the second camera in the world frame (literature convention)
+ **dir**:        name of the directory where to save the results

Output params:
+ **delta_R**:    rotation matrix of the extrinsics of the stereo pair with the first camera as reference (literature convention)
+ **delta_t**:    translation vector of the extrinsics of the stereo pair with the first camera as reference (literature convention)
+ **E**:          essential matrix of the stereo pair (literature convention)
+ **F**:          fundamental matrix of the stereo pair (literature convention)
</details>

<!-- calibration_extrinsics_stereo_smz matlab function -->
<details>
    <summary>
        calibration_extrinsics_stereo_smz
    </summary>

    [delta_R, delta_t, E, F] = calibration_extrinsics_stereo_smz(P1, K1, P2, K2, dir)

Input params:
+ **P1**:         cell array of projection matrices returned by SMZ calibration of the first camera (literature convention)
+ **K1**:         intrinsics matrix of the first camera (literature convention)
+ **P2**:         cell array of projection matrices returned by SMZ calibration of the second camera (literature convention)
+ **K2**:         intrinsics matrix of the second camera (literature convention)
+ **dir**:        name of the directory where to save the results

Output params:
+ **delta_R**:    rotation matrix of the extrinsics of the stereo pair with the first camera as reference (literature convention)
+ **delta_t**:    translation vector of the extrinsics of the stereo pair with the first camera as reference (literature convention)
+ **E**:          essential matrix of the stereo pair (literature convention)
+ **F**:          fundamental matrix of the stereo pair (literature convention)
</details>

<!-- calibration_intrinsics_camera matlab function -->
<details>
    <summary>
        calibration_intrinsics_camera
    </summary>

    [P, K, intrinsics] = calibration_intrinsics_camera(n_intrinsics, n_radial_dist, step_size, grid_arrangement, cm2px_scale, dir_images)

Input params:
+ **n_intrinsics**:       number of intrisics to be calibrated (4, 5)
    + 4: fx, fy, u0, v0
    + 5: fx, fy, u0, v0, skew
+ **n_radial_dist**:      number of the distortion coefficient to be calibrated (1, 2)
+ **step_size**:          side of the squares of the checkerboard [cm]
+ **grid_arrangement**:   [x-steps y-steps] steps of the checkerboard along x,y axes
+ **cm2px_scale**:        dimension in cm of 1 pixel of the rectified images   
+ **dir_images**:         path of the directory containing the checkerboard images  

Output params:
+ **P:                  cell array of projection matrices associated to the checkerboard images (literature convention)
+ **K:                  calibrated intrisics matrix (literature convention)
+ **intrinsics:         table with intrinsics and radial distortion parameters 
</details>

<!-- check_epipolar matlab function -->
<details>
    <summary>
        check_epipolar_geometry
    </summary>

    test = check_epipolar_geometry(cam1, cam2, F)

Input params:
+ **cam1**: camera object of the first camera (cf. webcam(...))
+ **cam2**: camera object of the second camera (cf. webcam(...))
+ **F**: fundamental matrix of the stereo pair (cam1 assumed as reference)

Output params:
+ **test**: value of p2'*F*p1 value where p1, p2 are the points acquired from the first and second camera respectively in homogeneous coordinates
</details>

<!-- check_svd matlab function -->
<details>
    <summary>
        check_svd
    </summary>

    sigma_svd = check_svd(X)

Input params:
+ **X**: cell array of candidated linearly dependent arrays

Output params:
+ **sigma_svd**: singolar values of the concatenated arrays
</details>

<!-- get_extrinsics_camera matlab function -->
<details>
    <summary>
        get_extrinsics_camera
    </summary>

    [R, t, G] = get_extrinsics_camera(P, K) 

Input params:
+ **P**: projective matrices (literature convention)
+ **K**: intrinsics matrices (literature convention)

Output params:
+ **R**: rotation matrices (literature convention)
+ **t**: translation vectors (literature convention)
+ **G**: roto-translation matrices (literature convention)
</details>

<!-- print_countdown matlab function -->
<details>
    <summary>
        print_countdown
    </summary>

    print_countdown(length)

Input params:
+ **length**: duration of the countdown [s]
</details>

<a name="robot-vision"></a>
## Robot Vision
<!-- aruco_detection matlab function -->
<details>
    <summary>
        aruco_detection
    </summary>

    [rois_matched, i_arucos] = aruco_detection(img, aruco_markers, varargin)

Input params:
+ **img**: input image
+ **ruco_markers**: markers to be matched
+ **varargin**: collection of optional parameters, cf. the official Matlab documentation

Output params:
+ **rois_matched**: ROIs matched with the markers
+ **i_arucos**: indices of the markers matched with the rois_matched
</details>

<!-- aruco_pose_estimation matlab function -->
<details>
    <summary>
        aruco_pose_estimation
    </summary>

    [rois, i_arucos, rois_R, rois_t] = aruco_pose_estimation(img, aruco_markers, aruco_real_sides, K, R_cam, t_cam, varargin)

Input params:
+ **img**:                input image
+ **aruco_markers**:      markers to be matched
+ **aruco_real_sides**:   real world lengths of the sides of the markers [cm]
+ **K**:                  intrisics matrix of the camera (Matlab convention)
+ **R_cam**:              rotation matrix of the camera extrinsics in the world frame (Matlab convention)
+ **t_cam**:              translation vector of the camera extrinsics in the world frame (Matlab convention)
+ **varargin**:           collection of optional parameters, cf. the official Matlab documentation

Output params:
+ **rois**:               ROIs matched with the markers
+ **i_arucos**:           indices of the markers matched with the rois
+ **rois_R**:             rotation matrices of the roto-translations that map points from the ROIs frames into the world frame (Matlab convention)
+ **rois_t**:             translation vectors of the roto-translations that map points from the ROIs frames into the world frame (Matlab convention)   
</details>

<!-- check_boundaries matlab function -->
<details>
    <summary>
        check_boundaries
    </summary>

    check_ans = check_boundaries(i, j, img_size)

Input params:
+ **i**:          i point coordinate (row)
+ **j**:          j point coordinate (column)
+ **img_size**:   1x2 [rows img limit, columns img limit]

Output params:
+ **check_ans**:  1 if the point is inside the image 0 otherwise
</details>

<!-- check_quadrilateral matlab function -->
<details>
    <summary>
        check_quadrilateral
    </summary>

    is_valid_quad = check_quadrilateral(points, varargin)

Input params:
+ **points**:   array Nx2 of points that defines the shape [ [x1,y1]; [x2,y2]; ... ; [xN,yN] ]
+ **varargin**: collection of optional parameters, cf. the official Matlab documentation

Output params:
+ **is_valid_quad**: 1 if the shape is a valid quadrilateral 0 otherwise
</details>

<!-- get_image matlab function -->
<details>
    <summary>
        get_image
    </summary>

    img = get_image(img_source)

Input params:
+ **img_source**: webcam object or path to an image on disk

Output params:
+ **img**: image acquired from the camera or loaded from disk
</details>

<!-- homography matlab function -->
<details>
    <summary>
        homography
    </summary>

    Y = homography(X, H)

Input params:
+ **X**: input set of points (inhomogeneous coordinates)
+ **H**: linear transformation between homogeneous coordinates (Matlab convention: hom(Y) = hom(X)*H)

Output params:
+ **Y**: transformed set of points (inhomogeneous coordinates)
</details>

<!-- plot_aruco_markers matlab function -->
<details>
    <summary>
        plot_aruco_markers
    </summary>

    plot_aruco_markers(aruco_markers)

Input params:
+ **aruco_markers**: cell array containing the Aruco markers
</details>

<!-- pnp_lin matlab function -->
<details>
    <summary>
        pnp_lin
    </summary>

    [R, t, reproj_err] = pnp_lin(X_image, X_world, K)
    
Input params:
+ **X_image**:    Nx2 array, 2D image points
+ **X_world**:    Nx3 array, 3D world points ( X_world(:,3) = 0 )
+ **K**:          intrisics matrix of the camera

Output params:
+ **R**:          rotation matrix of the camera extrinsics
+ **t**:          translation vector of the camera extrinsics
+ **reproj_err**: reprojection error (RMS value)

NOTE: Matlab convention is assumed, X_image = X_world*[R; t]*K.
</details>

<!-- pnp_nonlin matlab function -->
<details>
    <summary>
        pnp_nonlin
    </summary>

    [R, t, reproj_err] = pnp_nonlin(R0, t0, X_image, X_world, K)

Input params:
+ **R0**:         initial guess for the rotation matrix of the camera extrinsics, e.g., calculated with pnp_lin(...)
+ **t0**:         initial guess for the translation vector of the camera extrinsics, e.g., calculated with pnp_lin(...)
+ **X_image**:    Nx2 array, 2D image points
+ **X_world**:    Nx3 array, 3D world points
+ **K**:          intrisics matrix of the camera

Output params:
+ **R**:          rotation matrix of the (refined) camera extrinsics
+ **t**:          translation vector of the (refined) camera extrinsics
+ **reproj_err**: reprojection error (RMS value)

NOTE: Matlab convention is assumed, X_image = X_world*[R; t]*K.
</details>

<!-- reprojection_error matlab function -->
<details>
    <summary>
        reprojection_error
    </summary>

    [err, J_ext] = reprojection_error(m, M, K, R, t)

Input params:
+ **m**:      2D image point
+ **M**:      3D world point
+ **K**:      intrisics matrix of the camera
+ **R**:      rotation matrix of the camera extrinsics
+ **t**:      translation vector of the camera extrinsics

Output params:
+ **err**:    2x1 array, reprojection error between m and reproj(M)
+ **J_ext**:  2x12 array, Jacobian of err wrt the camera extrinsics [R11,R21,R31,R12,R22,R32,R13,R23,R33,t1,t2,t3]

NOTE: Matlab convention is assumed, reproj(M) = M*[R; t]*K.
</details>

<!-- roi_extraction matlab function -->
<details>
    <summary>
        roi_extraction
    </summary>

    [rois_raw, time] = roi_extraction(img, img_gray, varargin)

Input params:
+ **img**:      input image
+ **img_gray**: input image (grayscale)
+ **varargin**: collection of optional parameters, cf. the official Matlab documentation

Output params:
+ **rois_raw**: extracted ROIs without any refinement
+ **time**: execution time (ignoring plots)
</details>

<!-- roi_extraction_dfs matlab function -->
<details>
    <summary>
        roi_extraction_dfs
    </summary>

    components = roi_extraction_dfs(img_canny)

Input params:
+ **img_canny**: input image filtered by Canny edge detector

Output params:
+ **components**: cell array of the connected components (points and tails)
    + components{i,1} is the set of points of the i-th component
    + components{i,2} is the set of tails of the i-th component
</details>

<!-- roi_extraction_dfs_c matlab function -->
<details>
    <summary>
        roi_extraction_dfs_c
    </summary>

TODO
</details>

<!-- roi_matching matlab function -->
<details>
    <summary>
        roi_matching
    </summary>

    [rois_matched, i_rois_matched, i_arucos, time] = roi_matching(img, img_gray, rois, aruco_markers, varargin)

Input params:
+ **img**: input image
+ **img_gray**: input image (grayscale)
+ **rois**: candidated ROIs for matching with markers
+ **aruco_markers**: markers to be matched
+ **varargin**: collection of optional parameters, cf. the official Matlab documentation

Output params:
+ **rois_matched**: matched ROIs among the candidated ROIs
+ **i_rois_matched**: indices of the rois_matched in the rois cell array
+ **i_arucos**: indices of the markers matched with the rois_matched
+ **time**: execution time (ignoring plots)
</details>

<!-- roi_pose_estimation matlab function -->
<details>
    <summary>
        roi_pose_estimation
    </summary>

    [R, t, err_lin, err_nonlin, time] = roi_pose_estimation(img, rois, i_arucos, aruco_real_sides, K, R_cam, t_cam, varargin)

Input params:
+ **img**: input image
+ **rois**: ROIs matched with the markers
+ **i_arucos**: indices of the matched markers for every ROIs
+ **aruco_real_sides**: real world lengths of the sides of the markers [cm]
+ **K**: intrisics matrix of the camera (Matlab convention)
+ **R_cam**: rotation matrix of the camera extrinsics in the world frame (Matlab convention)
+ **t_cam**: translation vector of the camera extrinsics in the world frame (Matlab convention)
+ **varargin**: collection of optional parameters, cf. the official Matlab documentation

Output params:
+ **R**: rotation matrices of the roto-translations that map points from the roi frames into the world frame (Matlab convention)
+ **t**: translation vectors of the roto-translations that map points from the roi frames into the world frame (Matlab convention)
+ **err_lin**: RMS values of reprojection errors (after linear PnP)
+ **err_nonlin**: RMS values of reprojection errors (after non-linear PnP)
+ **time**: execution time (ignoring plots)
</details>

<!-- roi_refinement matlab function -->
<details>
    <summary>
        roi_refinement
    </summary>

    [rois_refined, i_rois_refined, time] = roi_refinement(img, rois_raw, varargin)

Input params:
+ **img**: input image
+ **rois_raw**: input ROIs without any refinement
+ **varargin**: collection of optional parameters, cf. the official Matlab documentation

Output params:
+ **rois_refined**: refined and selected ROIs among the input ROIs
+ **i_rois_refined**: indices of the rois_refined in the rois_raw cell array
+ **time**: execution time (ignoring plots)
</details>

<!-- rpy2rot matlab function -->
<details>
    <summary>
        rpy2rot
    </summary>

    [R, J_roll, J_pitch, J_yaw] = rpy2rot(a)

Input params:
+ **a**: roll-pitch-yaw parameterization of the rotation
    + a(1) = roll  (rotation angle around x-axis)
    + a(2) = pitch (rotation angle around y-axis)
    + a(3) = yaw   (rotation angle around z-axis)

Output params:
+ **R**: rotation matrix, R = Rx(roll)*Ry(pitch)*Rz(yaw)
+ **J_roll**: Jacobian of R wrt roll
+ **J_pitch**: Jacobian of R wrt pitch
+ **J_yaw**: Jacobian of R wrt yaw
</details>

<a name="robot-control"></a>
## Robot Control