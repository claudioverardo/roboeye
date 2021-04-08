# Documentation

1. [Robot Calibration](#robot-calibration)
2. [Robot Vision](#robot-vision)
3. [Robot Trajectory Planning](#robot-trajectory-planning)
4. [Robot Control](#robot-control)

<a name="robot-calibration"></a>
## Robot Calibration

Code to perform the calibration of the cameras and the robot with a checkerboard.

These functions assume the usual conventions to represent the pinhole model of a camera (in the following, **literature convention**). Namely, points on the image plane `m` and in space `M` are represented by column vectors. The projection equation (cf. the references of the project) takes the form `m = P*M = K*[R t]*M`, where `P` is the projection matrix, `K` the intrinsics matrix, and `[R t]` the extrinsics of the camera.

### Functions

<!-- acquire_calibration_images matlab function -->
<details>
    <summary>
        acquire_calibration_images
    </summary>

Acquire some images of a checkerboard from a set of fixed cameras at the same time. These images can be used to calibrate the cameras with the SMZ algorithm.

    images = acquire_calibration_images(n_images, cameras, dirs_images)

Input arguments:
+ **n_images**: number of the images to be acquired from each camera
+ **cameras**: cell array of camera objects (cf. webcam(...))
+ **dirs_images**: cell array with the directory paths where to save the images

Output arguments:
+ **images**: cell array of acquired images. images{i,j} is the i-th image acquired from the j-th camera

NOTE: this function requires the MATLAB Support Package for USB Webcams.
</details>

<!-- calibration_extrinsics_camera matlab function -->
<details>
    <summary>
        calibration_extrinsics_camera
    </summary>

Retrive the rotation matrix and the translation vector (extrinsics) of a camera wrt a world frame attached to a checkerboard.

    [R_cam, t_cam] = calibration_extrinsics_camera(cam, K, k, step_size, grid_arrangement, cm2px_scale, dir)

Input arguments:
+ **cam**:                webcam object (cf. webcam(...))
+ **K**:                  intrinsics matrix of the camera (literature convention)
+ **k**:                  radial distortion coefficients of the camera
+ **step_size**:          side of the squares of the checkerboard [cm]
+ **grid_arrangement**:   [x-steps y-steps] steps of the checkerboard along x,y axes
+ **cm2px_scale**:        dimension in cm of 1 pixel of the rectified image
+ **dir**:                directory where to write/read the calibration files

Output arguments:
+ **R_cam**: rotation matrix of the camera extrinsics in the world frame (literature convention)
+ **t_cam**: translation vector of the camera extrinsics in the world frame (literature convention)

NOTE: this function requires the following packages:
+ MATLAB Support Package for USB Webcams
+ Computer Vision Toolkit (http://www.diegm.uniud.it/fusiello/demo/toolkit/)
</details>

<!-- calibration_extrinsics_stereo matlab function -->
<details>
    <summary>
        calibration_extrinsics_stereo
    </summary>

Retrieve the extrinsics and epipolar matrices of a stereo pair. The two cameras are assumed with known intrinsics and extrinsics wrt the same world frame.

    [delta_R, delta_t, E, F] = calibration_extrinsics_stereo(K1, R1, t1, K2, R2, t2, dir)

Input arguments:
+ **K1**:         intrinsics matrix of the first camera (literature convention)
+ **R1**:         rotation matrix of the extrinsics of the first camera in the world frame (literature convention)
+ **t1**:         translation vector of the extrinsics of the first camera in the world frame (literature convention)
+ **K2**:         intrinsics matrix of the second camera (literature convention)
+ **R2**:         rotation matrix of the extrinsics of the second camera in the world frame (literature convention)
+ **t2**:         translation vector of the extrinsics of the second camera in the world frame (literature convention)
+ **dir**:        name of the directory where to save the results

Output arguments:
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

Retrieve the extrinsics and epipolar matrices of a stereo pair. The two cameras are assumed to be previously jointly calibrated with the SMZ algorithm and with fixed relative position afterwards.

    [delta_R, delta_t, E, F] = calibration_extrinsics_stereo_smz(P1, K1, P2, K2, dir)

Input arguments:
+ **P1**:         cell array of projection matrices returned by SMZ calibration of the first camera (literature convention)
+ **K1**:         intrinsics matrix of the first camera (literature convention)
+ **P2**:         cell array of projection matrices returned by SMZ calibration of the second camera (literature convention)
+ **K2**:         intrinsics matrix of the second camera (literature convention)
+ **dir**:        name of the directory where to save the results

Output arguments:
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

Retrive the intrisics and radial distortion parameters of a camera using a set of checkerboard images (SMZ algorithm).

    [P, K, intrinsics] = calibration_intrinsics_camera(n_intrinsics, n_radial_dist, step_size, grid_arrangement, cm2px_scale, dir_images)

Input arguments:
+ **n_intrinsics**:       number of intrisics to be calibrated (4, 5)
    + 4: fx, fy, u0, v0
    + 5: fx, fy, u0, v0, skew
+ **n_radial_dist**:      number of the distortion coefficient to be calibrated (1, 2)
+ **step_size**:          side of the squares of the checkerboard [cm]
+ **grid_arrangement**:   [x-steps y-steps] steps of the checkerboard along x,y axes
+ **cm2px_scale**:        dimension in cm of 1 pixel of the rectified images   
+ **dir_images**:         path of the directory containing the checkerboard images  

Output arguments:
+ **P**:                cell array of projection matrices associated to the checkerboard images (literature convention)
+ **K**:                calibrated intrisics matrix (literature convention)
+ **intrinsics**:       table with intrinsics and radial distortion parameters 

NOTE: this function requires the Computer Vision Toolkit (http://www.diegm.uniud.it/fusiello/demo/toolkit/)
</details>

<!-- check_epipolar matlab function -->
<details>
    <summary>
        check_epipolar_geometry
    </summary>

Acquire two points from the two images of a stereo pair and compute the Longuet-Higgins equation between them.

    test = check_epipolar_geometry(cam1, cam2, F)

Input arguments:
+ **cam1**: camera object of the first camera (cf. webcam(...))
+ **cam2**: camera object of the second camera (cf. webcam(...))
+ **F**: fundamental matrix of the stereo pair (cam1 assumed as reference)

Output arguments:
+ **test**: value of the Longuet-Higgins equation `p2'*F*p1`, where `p1`, `p2` are the points acquired from the first and second camera respectively (in homogeneous coordinates)
</details>

<!-- check_svd matlab function -->
<details>
    <summary>
        check_svd
    </summary>

SVD test to check if arrays are (numerically) linearly dependent.

    sigma_svd = check_svd(X)

Input arguments:
+ **X**: cell array of candidated linearly dependent arrays

Output arguments:
+ **sigma_svd**: singolar values of the concatenated arrays
</details>

<!-- get_extrinsics_camera matlab function -->
<details>
    <summary>
        get_extrinsics_camera
    </summary>

Retrieve the extrinsics of a set of cameras from their projection matrices and intrinsics matrices.

    [R, t, G] = get_extrinsics_camera(P, K) 

Input arguments:
+ **P**: cell array of projection matrices (literature convention)
+ **K**: cell array of intrinsics matrices (literature convention)

Output arguments:
+ **R**: cell array of rotation matrices (literature convention)
+ **t**: cell array of translation vectors (literature convention)
+ **G**: cell array of `[R t; 0 1]` matrices (literature convention)
</details>

<!-- print_countdown matlab function -->
<details>
    <summary>
        print_countdown
    </summary>

Plot on the screen the countdown of length seconds.

    print_countdown(length)

Input arguments:
+ **length**: duration of the countdown [s]
</details>

Usage examples can be found in [run_calibration_camera](../src/scripts/run_calibration_camera.m) and [run_calibration_stereo](../src/scripts/run_calibration_stereo.m).

<a name="robot-vision"></a>
## Robot Vision

Code to perform the detection and the pose estimation of Aruco markers in the scene.

These functions assume the Matlab conventions to represent the pinhole model of a camera (in the following, **Matlab convention**). Namely, points on the image plane `m` and in space `M` are represented by row vectors. The projection equation takes the form `m = M*P = M*[R; t]*P`, where `P, K, R, t` are the transposes of their counterparts in the literature convention.

We refer to **world frame** as the coordinate system XYZ wrt the camera extrinsics are calibrated (magenta-cyan-yellow axes in the image below). We refer to **ROI frame** (also, ROI pose) as the coordinate system xyz placed at the center of a marker such that its contol points is at `(-0.5,0.5,0)*side`, where `side` is the real length of the marker side (red-blue-green axes in the image below).

![Frames](./frames.png)

### Functions

<!-- aruco_detection matlab function -->
<details>
    <summary>
        aruco_detection
    </summary>

Build the Aruco detection pipeline. It executes in order the functions roi_extraction(...), roi_refinement(...), roi_matching(...).

    [rois_matched, i_arucos, stats] = aruco_detection(img, aruco_markers, varargin)

Input arguments:
+ **img**: input image
+ **ruco_markers**: markers to be matched
+ **varargin**: collection of optional parameters, cf. the official Matlab documentation

Parameters:
+ **'verbose'**: verbose level of the function (0, 1)
    + 0: show nothing
    + 1: show log in the command window
+ Refer to roi_extraction(...), roi_refinement(...), roi_matching(...) for details about further allowed parameters.

Output arguments:
+ **rois_matched**: ROIs matched with the markers
+ **i_arucos**: indices of the markers matched with the rois_matched
+ **stats**: struct with some performance statistics
    + number of ROIs extracted/refined
    + times of ROIs extraction/refinement/matching
</details>

<!-- aruco_pose_estimation matlab function -->
<details>
    <summary>
        aruco_pose_estimation
    </summary>

Build the Aruco pose estimation pipeline. It executes in order the functions aruco_detection(...), roi_pose_estimation(...).

    [rois, i_arucos, rois_R, rois_t] = aruco_pose_estimation(img, aruco_markers, aruco_real_sides, K, R_cam, t_cam, k, varargin)

Input arguments:
+ **img**:                input image
+ **aruco_markers**:      markers to be matched
+ **aruco_real_sides**:   real world lengths of the sides of the markers [cm]
+ **K**:                  intrisics matrix of the camera (Matlab convention)
+ **R_cam**:              rotation matrix of the camera extrinsics in the world frame (Matlab convention)
+ **t_cam**:              translation vector of the camera extrinsics in the world frame (Matlab convention)
+ **k**:                  radial distortion coefficients of the camera
+ **varargin**:           collection of optional parameters, cf. the official Matlab documentation

Parameters:
+ **'verbose'**: verbose level of the function (0, 1)
    + 0: show nothing
    + 1: show log in the command window
+ Refer to aruco_detection(...), roi_pose_estimation(...) for details about further allowed parameters

Output arguments:
+ **rois**:               ROIs matched with the markers
+ **i_arucos**:           indices of the markers matched with the rois
+ **rois_R**:             rotation matrices of the roto-translations that map points from the ROIs frames into the world frame (Matlab convention)
+ **rois_t**:             translation vectors of the roto-translations that map points from the ROIs frames into the world frame (Matlab convention)
+ **stats**:              struct with some performance statistics
    + number of ROIs extracted/refined
    + times of ROIs extraction/refinement/matching and pose estimation    
    + reprojection errors of lin/nonlin PnP
</details>

<!-- check_boundaries matlab function -->
<details>
    <summary>
        check_boundaries
    </summary>

Return 1 if the point (i, j) in pixel coordinates is inside an image, whose size is defined by img_size.

    check_ans = check_boundaries(i, j, img_size)

Input arguments:
+ **i**:          i point coordinate (row)
+ **j**:          j point coordinate (column)
+ **img_size**:   1x2 [rows img limit, columns img limit]

Output arguments:
+ **check_ans**:  1 if the point is inside the image 0 otherwise
</details>

<!-- check_quadrilateral matlab function -->
<details>
    <summary>
        check_quadrilateral
    </summary>

Check if the set of input points defines the shape of a valid quadrilateral, i.e., it is close to the shape of a parallelogram.

    is_valid_quad = check_quadrilateral(points, varargin)

Input arguments:
+ **points**:   array Nx2 of points that defines the shape [ [x1,y1]; [x2,y2]; ... ; [xN,yN] ]
+ **varargin**: collection of optional parameters, cf. the official Matlab documentation

Parameters:
+ **'sum_angles_tol'**: tolerance on the sum of the internal angles [degrees]
+ **'parallelism_tol'**: tolerance on the angle between opposite sides [degrees]
+ **'side_th_low'**: lower threshold on the length of each side [pixels]
+ **'side_th_high'**: higher threshold on the length of each side [pixels]
+ **'angle_th_low'**: lower threshold on the internal angles [degrees]
+ **'angle_th_high'**: higher threshold on the internal angles [degrees]

Output arguments:
+ **is_valid_quad**: 1 if the shape is a valid quadrilateral 0 otherwise

NOTE: a shape is discarded when one of the following conditions is met:
+ sum of the internal angles > 360° + sum_angles_tol 
+ angle between opposide sides > parallelism_tol
+ length of a side < side_th_low
+ length of a side > side_th_high
+ value of an internal < angle_th_low
+ value of an internal > angle_th_high
</details>

<!-- get_image matlab function -->
<details>
    <summary>
        get_image
    </summary>

Acquire an image from camera or load an image from disk.

    img = get_image(img_source)

Input arguments:
+ **img_source**: webcam object or path to an image on disk

Output arguments:
+ **img**: image acquired from the camera or loaded from disk
</details>

<!-- hom_tf matlab function -->
<details>
    <summary>
        hom_tf
    </summary>

Apply the homogeneous transformation H to the set of points X. The points are arranged by rows X = [x1; ... ; xN] and Y = [y1; ... ; yN]. The transformation acts on the homogeneous coordinates, hom(Y) = hom(X)*H. If required, apply also radial distortion to the results.

    Y = hom_tf(X, H)

    Y = hom_tf(X, H, K, k)

Input arguments:
+ **X**: input set of points (inhomogeneous coordinates)
+ **H**: transformation between homogeneous coordinates (Matlab convention)
    + H 4x3 is a projection 
    + H 3x3 is a transformation in the projective plane
    + H 4x4 is a transformation in the projective space
+ **K**: intrisics matrix of the camera (optional)
+ **k**:  radial distortion coefficients of the camera (optional)

Output arguments:
+ **Y**: transformed set of points (inhomogeneous coordinates)

NOTE: if intrinsics K and radial distortion coefficients k are provided, the points Y must be 2D image points.
</details>

<!-- plot_aruco_markers matlab function -->
<details>
    <summary>
        plot_aruco_markers
    </summary>

Show the content of a set of Aruco markers.

    plot_aruco_markers(aruco_markers)

Input arguments:
+ **aruco_markers**: cell array containing the Aruco markers
</details>

<!-- pnp_lin matlab function -->
<details>
    <summary>
        pnp_lin
    </summary>

Perspective-n-Points (PnP) from 3D-2D correspondences. It finds the camera extrinsics R, t wrt the frame of the 3D points from a set of 3D-2D correspondences. The algorithm assumes coplanar 3D points, i.e. with z=0. Also the RMS value of the reprojection errors is returned.

    [R, t, reproj_err] = pnp_lin(X_image, X_world, K)
    
Input arguments:
+ **X_image**:    Nx2 array, 2D image points
+ **X_world**:    Nx3 array, 3D world points ( X_world(:,3) = 0 )
+ **K**:          intrisics matrix of the camera

Output arguments:
+ **R**:          rotation matrix of the camera extrinsics
+ **t**:          translation vector of the camera extrinsics
+ **reproj_err**: reprojection error (RMS value)

NOTE: Matlab convention is assumed, `X_image = X_world*[R; t]*K`.
</details>

<!-- pnp_nonlin matlab function -->
<details>
    <summary>
        pnp_nonlin
    </summary>

Non-linear refinement of Perspective-n-Points (PnP) from 3D-2D correspondences. It iterativelly refines the input camera extrinsics through inimization of the reprojection errors of a set of 3D-2D correspondences. Also the RMS value of the final reprojection errors is returned.

    [R, t, reproj_err] = pnp_nonlin(R0, t0, X_image, X_world, K, k)

Input arguments:
+ **R0**:         initial guess for the rotation matrix of the camera extrinsics, e.g., calculated with pnp_lin(...)
+ **t0**:         initial guess for the translation vector of the camera extrinsics, e.g., calculated with pnp_lin(...)
+ **X_image**:    Nx2 array, 2D image points
+ **X_world**:    Nx3 array, 3D world points
+ **K**:          intrisics matrix of the camera
+ **k**:          radial distortion coefficients of the camera

Output arguments:
+ **R**:          rotation matrix of the (refined) camera extrinsics
+ **t**:          translation vector of the (refined) camera extrinsics
+ **reproj_err**: reprojection error (RMS value)

NOTE: Matlab convention is assumed, `X_image = fd( X_world*[R; t]*K)` where `fd` is the function that applies the radial distortion.
</details>

<!-- rad_dist_apply matlab function -->
<details>
    <summary>
        rad_dist_apply
    </summary>

Return the distorted pixel coordinates from the true ones.

    [m_d, J_m] = rad_dist_apply(m, K, k)

Input arguments:
+ **m**:      Nx2 array, undistorted image points
+ **K**:      intrisics matrix of the camera (Matlab convention)
+ **k**:      radial distortion coefficients of the camera

Output arguments:
+ **m_d**:    Nx2 array, distorted image points
+ **J_m**:    cell array of Jacobians of m_d wrt m (2x2 matrices)
</details>

<!-- rad_dist_remove matlab function -->
<details>
    <summary>
        rad_dist_remove
    </summary>

Return the true pixel coordinates from the distorted ones (solving a non-linear iterative LS problem).

    [m, err] = rad_dist_remove(m_d, K, k)

Input arguments:
+ **m_d**:    Nx2 array, distorted image points
+ **K**:      intrisics matrix of the camera (Matlab convention)
+ **k**:      radial distortion coefficients of the camera

Output arguments:
+ **m**:      Nx2 array, undistorted image points
+ **err**:    final error of the iterative solver (RMS value)
</details>

<!-- reprojection_error matlab function -->
<details>
    <summary>
        reprojection_error
    </summary>

Reprojection error of a 3D-2D correspondence. It finds the component-wise reprojection error between a 2D point and a 3D point. The Jacobian wrt the extrinsics of the camera is also returned.

    [err, J_ext] = reprojection_error(m, M, K, R, t, k)

Input arguments:
+ **m**:      Nx2 array, 2D image points
+ **M**:      Nx3 array, 3D world points
+ **K**:      intrisics matrix of the camera
+ **R**:      rotation matrix of the camera extrinsics
+ **t**:      translation vector of the camera extrinsics
+ **k**:      radial distortion coefficients of the camera

Output arguments:
+ **err**:    2Nx1 array, reprojection error between `m` and `reproj(M)`
+ **J_ext**:  2Nx12 array, Jacobian of err wrt the camera extrinsics `[R11,R21,R31,R12,R22,R32,R13,R23,R33,t1,t2,t3]`

NOTE: Matlab convention is assumed, `reproj(M) = fd( M*[R; t]*K )` where `fd` is the function that applies the radial distortion.
</details>

<!-- roi_extraction matlab function -->
<details>
    <summary>
        roi_extraction
    </summary>

Extract ROIs from the input image.

    [rois_raw, time] = roi_extraction(img, img_gray, varargin)

Input arguments:
+ **img**:      input image
+ **img_gray**: input image (grayscale)
+ **varargin**: collection of optional parameters, cf. the official Matlab documentation

Parameters:
+ **'method'**: choose the ROI extraction algorithm
    + 'adaptth-moore': adaptive thresholding + Moore-Neighbor tracing 
    + 'canny-dfs': Canny edge detector + DFS
    + 'canny-dfs-c': Canny edge detector + DFS C-implementation   
+ **'adaptth_sensitivity'**: sensitivity of the adaptive thresholding, cf. adaptthresh(...)
+ **'adaptth_statistic'**:	statistic of the adaptive thresholding, cf. adaptthresh(...)		
+ **'adaptth_neighborhood'**:	neighborhood size of the adaptive thresholding, cf. adaptthresh(...)			
+ **'canny_th_low'**: lower threshold of the Canny edge detector, cf. edge(...)
+ **'canny_th_high'**: higher threshold of the Canny edge detector, cf. edge(...)		
+ **'verbose'**: verbose level of the function (0, 1, 2)
    + 0: show nothing
    + 1: show the extracted ROIs
    + 2: show also the binarized image (if 'adaptth-moore') or the Canny+DFS output (if 'canny-dfs')

Output arguments:
+ **rois_raw**: extracted ROIs without any refinement
+ **time**: execution time (ignoring plots)
</details>

<!-- roi_extraction_dfs matlab function -->
<details>
    <summary>
        roi_extraction_dfs
    </summary>

Apply the depth-first search (DFS) algorithm to a picture filtered with the Canny edge detector (2D-Graph) and extract the connected components from it (set of points, set of tails).

    components = roi_extraction_dfs(img_canny)

Input arguments:
+ **img_canny**: input image filtered by Canny edge detector

Output arguments:
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

Match the Aruco markers with the candidate ROIs.

    [rois_matched, i_rois_matched, i_arucos, time] = roi_matching(img, img_gray, rois_refined, i_rois_refined, aruco_markers, varargin)

Input arguments:
+ **img**: input image
+ **img_gray**: input image (grayscale)
+ **rois_refined**: candidated ROIs for matching with markers
+ **i_rois_refined**: indices of the rois_refined in the rois_raw cell array
+ **aruco_markers**: markers to be matched
+ **varargin**: collection of optional parameters, cf. the official Matlab documentation

Parameters:
+ **'roi_bb_padding'**: padding value of bounding boxes [pixels]
+ **'roi_h_side'**: side value of a ROI after homography [pixels]
+ **'roi_hamming_th'**: maximum value of hamming distance to detect a marker
+ **'verbose'**: verbose level of the function (0, 1, 2, 3)
    + 0: show nothing
    + 1: show the matched ROIs and the markers IDs
    + 2: show also the homographies of the matched ROIs
    + 3: show also the homographies of the unmatched ROIs

Output arguments:
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

Compute the poses of the matched ROIs in the world frame.

    [R, t, err_lin, err_nonlin, time] = roi_pose_estimation(img, rois, i_arucos, aruco_real_sides, K, R_cam, t_cam, k, varargin)

Input arguments:
+ **img**: input image
+ **rois**: ROIs matched with the markers
+ **i_arucos**: indices of the matched markers for every ROIs
+ **aruco_real_sides**: real world lengths of the sides of the markers [cm]
+ **K**: intrisics matrix of the camera (Matlab convention)
+ **R_cam**: rotation matrix of the camera extrinsics in the world frame (Matlab convention)
+ **t_cam**: translation vector of the camera extrinsics in the world frame (Matlab convention)
+ **k**: radial distortion coefficients of the camera
+ **varargin**: collection of optional parameters, cf. the official Matlab documentation

Parameters:
+ **'verbose'**: verbose level of the function (0, 1, 2)
    + 0: show nothing
    + 1: show the poses of the ROIs
    + 2: show also markers the IDs

Output arguments:
+ **R**: rotation matrices of the roto-translations that map points from the ROIs frames into the world frame (Matlab convention)
+ **t**: translation vectors of the roto-translations that map points from the ROIs frames into the world frame (Matlab convention)
+ **err_lin**: RMS values of reprojection errors (after linear PnP)
+ **err_nonlin**: RMS values of reprojection errors (after non-linear PnP)
+ **time**: execution time (ignoring plots)
</details>

<!-- roi_refinement matlab function -->
<details>
    <summary>
        roi_refinement
    </summary>

Refine and select the candidate ROIs for matching.

    [rois_refined, i_rois_refined, time] = roi_refinement(img, rois_raw, varargin)

Input arguments:
+ **img**: input image
+ **rois_raw**: input ROIs without any refinement
+ **varargin**: collection of optional parameters, cf. the official Matlab documentation

Parameters:
+ **'method'**: choose the ROI refinement algorithm
    + 'rdp': Ramer-Douglas–Peucker 
    + 'geometric': find the four extreme corners
+ **'roi_size_th'**: min #points required by each ROI to be processed
+ **'rdp_th'**: threshold of the Ramer-Douglas–Peucker algorithm, cf. reducepoly(...)
+ **'roi_sum_angles_tol'**: tolerance on the sum of the internal angles, cf. check_quadrilateral(...)
+ **'roi_parallelism_tol'**: tolerance on the angle between opposite sides, cf. check_quadrilateral(...)
+ **'roi_side_th_low'**: lower threshold on the length of each side normalized wrt the diagonal of the input image, cf. check_quadrilateral(...)
+ **'roi_side_th_high'**: higher threshold on the length of each side normalized wrt the diagonal of the input image, cf. check_quadrilateral(...)
+ **'roi_angle_th_low'**: lower threshold on the internal angles, cf. check_quadrilateral(...)
+ **'roi_angle_th_high'**: higher threshold on the internal angles, cf. check_quadrilateral(...)
+ **'verbose'**: verbose level of the function (0, 1, 2)
    + 0: show nothing
    + 1: show the refined ROIs
    + 2: show also the discarded ROIs

Output arguments:
+ **rois_refined**: refined and selected ROIs among the input ROIs
+ **i_rois_refined**: indices of the rois_refined in the rois_raw cell array
+ **time**: execution time (ignoring plots)

NOTE: to use Ramer-Douglas–Peucker ('rdp') Matlab >= 2019b is needed.
</details>

<!-- rpy2rot matlab function -->
<details>
    <summary>
        rpy2rot
    </summary>

Create a rotation matrix from its roll-pitch-yaw parameterization.

    [R, J_roll, J_pitch, J_yaw] = rpy2rot(a)

Input arguments:
+ **a**: [roll pitch yaw] parameterization of the rotation
    + a(1) = roll, rotation angle around x-axis
    + a(2) = pitch, rotation angle around y-axis
    + a(3) = yaw, rotation angle around z-axis

Output arguments:
+ **R**: rotation matrix, `R = Rx(roll)*Ry(pitch)*Rz(yaw)`
+ **J_roll**: Jacobian of R wrt roll
+ **J_pitch**: Jacobian of R wrt pitch
+ **J_yaw**: Jacobian of R wrt yaw
</details>

Usage examples can be found in [run_detection](../src/scripts/run_detection.m) and [run_pose_estimation](../src/scripts/run_pose_estimation.m).

<a name="robot-trajectory-planning"></a>
## Robot Trajectory Planning

Code to perform direct kinematics, inverse kinematics and trajectory planning.

### Functions

TODO

<a name="robot-control"></a>
## Robot Control

Code to perform the control of the robot with Arduino and the Matlab interface.

### Arduino files

<!-- arduino_robot_fsm Arduino code -->
<details>
    <summary>
        arduino_robot_fsm
    </summary>

Implementation of the finite state machine (FSM) that control the robot. The Matlab interface is provided by the function robot_fsm_interface(...).
</details>

### Functions

<!-- cmd_acquire matlab function -->
<details>
    <summary>
        cmd_acquire
    </summary>

Acquire a command that satisfy a given validation function. Both manual input from user and automatic input from buffer are supported.

    cmd = cmd_acquire(help, fn_val, fn_robot_input, cmd_ask_str, cmd_not_valid_str)

Input arguments:
+ **help**: help message to be displayed before acquisition
+ **fn_val**: validation function of the command
+ **fn_robot_input**: function to acquire input, cf. input(...) or cmdBuffer
+ **cmd_ask_str**: message to require a command (optional)
+ **cmd_not_valid_str**: message if the acquired command is invalid (optional)

Output arguments:
+ **cmd**: command acquired
</details>

<!-- cmd_execute matlab function -->
<details>
    <summary>
        cmd_execute
    </summary>

Execute a command on Arduino and wait for an acknowledge.

    cmd_err = cmd_execute(s, cmd, data_tx, cmd_ack_str, cmd_nack_str)

Input arguments:
+ **s**: object of the Arduino serial port, cf. serialport(...) 
+ **cmd**: command to be executed  
+ **data_tx**: data associated to the command
+ **cmd_ack_str**: ACK message (optional)
+ **cmd_nack_str**: missing ACK message (optional)

Output arguments:
+ **cmd_err**: 1 if ACK is missing, 0 otherwise
</details>

<!-- fix_target_q matlab function -->
<details>
    <summary>
        fix_target_q
    </summary>

Add a small overshoot to the trajectory of the first joint during the movement to a target position.

    target_q_fix = fix_target_q(target_q, current_q, last_q)

Input arguments:
+ **target_q**: 1xQNUM array, target position in joints space
+ **current_q**: 1xQNUM array, current position in joints space
+ **last_q**: 1xQNUM array, last position in joints space

Output arguments:
+ **target_q_fix**: 3xQNUM array, fixed target position
</details>

<!-- generate_trajectory matlab function -->
<details>
    <summary>
        generate_trajectory
    </summary>

High level interface to generate robot trajectories. Trajectories defined pointwise (P) and via keypoints (K) can be generated. The latter ones require a low level controller that interpolate between keypoints to be executed on the robot.

    [trajectory, time_trajectory, confirm] = generate_trajectory(method, current_q, delta_t, cam, vision_args, trajectory_planning_args, fn_cam2robot_coords, fn_robot_input)

Input arguments:
+ **method**: method used to generate the trajectory
    + 'back-home': go back to the home position (K)
    + 'move-q': move to a position in joints space (K)
    + 'move-t-npoints': move to n positions in 3D space (K)
    + 'move-t-pointwise': move to a position in 3D space from home (P)
    + 'move-t': move to a position in 3D space (K)
    + 'grasp': grasp a object in a position in 3D space (K)
    + 'grasp-parabola': as 'grasp', with a parabolic trajectory (K)
+ **home_q**: 1xQNUM array, home position of the robot (joints)
+ **current_q**:1xQNUM array, current position of the robot (joints)
+ **delta_t**: timestep of the trajectory execution
+ **cam**: webcam object of the camera, cf. webcam(...)
+ **vision_args**: struct of vision parameters
+ **trajectory_planning_args**: struct of trajectory planning parameters
+ **fn_cam2robot_coords**: function to convert points from vision to robot frame 
+ **fn_robot_input**: function to acquire input, cf. input(...) or cmdBuffer

trajectory_planning_args struct:
+ braccio_params: parameters of the robot
+ z_min: minimum z-value of target points [mm], in robot frame
+ box_coords_grasp: destination of 'grasp' [cm], in vision frame
+ box_coords_grasp_parabola: as above but for 'grasp-parabola' [cm]
+ touchdown_verbose: verbosity level of touchdown(...)
+ gothere_verbose: verbosity level of gothere(...)
+ parabolic_traj_verbose: verbosity level of parabolic_traj(...)
+ objects_dict: parameters of the objects to be grasped, cf. object_offset(...)

Output arguments:
+ **trajectory**: NxQNUM array of the generated N-points trajectory
+ **time_trajectory**: estimated execution time of the trajectory
+ **confirm**: flag to confirm or cancel execution of the trajectory

NOTE: this function requires the MATLAB Support Package for USB Webcams. For details regarding vision_args refers to get_target_from_vision(...).
</details>

<!-- get_target matlab function -->
<details>
    <summary>
        get_target
    </summary>

Retrieve the position of a target in the scene (world frame).

    [target, i_aruco] = get_target(method, QNUM, cam, vision_args, fn_robot_input)

Input arguments:
+ **method**: type of target acquisition
    + 'q': position in joint space from user
    + '3d-npoints': n positions in 3d space from user (world frame)
    + '3d-vision': position in 3d space from user or camera (world frame)
+ **QNUM**: number of joints of the robot
+ **cam**: webcam object of the camera, cf. webcam(...)
+ **vision_args**: struct of vision parameters
+ **fn_robot_input**: function to acquire input, cf. input(...) or cmdBuffer

Output arguments:
+ **target**: position ot the chosen target (world frame)
+ **i_aruco**: id of the marker associated to the target (0 if none)

NOTE: this function requires the MATLAB Support Package for USB Webcams. For details regarding vision_args refers to get_target_from_vision(...).
</details>

<!-- get_target_from_vision matlab function -->
<details>
    <summary>
        get_target_from_vision
    </summary>

Retrieve the position of a chosen marker in the scene observed by a camera (world frame).

    [t, R, i_aruco] = GET_TARGET_FROM_VISION(cam, vision_args, fn_robot_input)

Input arguments:
+ **cam**: webcam object of the camera, cf. webcam(...)
+ **vision_args**: struct of vision parameters, cf. below
+ **fn_robot_input**: function to acquire input, cf. input(...) or cmdBuffer

The struct vision_args contains the positional arguments and parameters of aruco_pose_estimation(...).

Output arguments:
+ **t**: translation vector of the roto-translation that maps points from the target frame into the world frame (Matlab convention)
+ **R**: rotation matrix of the roto-translation that maps points from the target frame into the world frame (Matlab convention)
+ **i_aruco**: id of the marker correspondent to the target
</details>

<!-- estimate_time_trajectory matlab function -->
<details>
    <summary>
        estimate_time_trajectory
    </summary>

Estimate the time to execute a trajectory on the robot.

    time = estimate_time_trajectory(type_trajectory, trajectory, current_q, delta_t)

Input arguments:
+ **type_trajectory**: type of trajectory, cf. generate_trajectory(...)
    + 'pointwise': trajectory defined point by point
    + 'keypoints': trajectory defined via keypoints to be interpolated
+ **trajectory**: NxQNUM array, points of the trajectory
+ **current_q**: 1xQNUM array, current position of the robot (joints)
+ **delta_t**: timestep of the trajectory execution

Output arguments:
+ **time**: estimated execution time of the trajectory
</details>

<!-- object_offset matlab function -->
<details>
    <summary>
        object_offset
    </summary>

Find the offset of the end-effector position to adjust the grasping position on the basis of object data (in robot coordinates).

    function dt = object_offset(dh, dr, t, nz)

Input arguments:
+ **dh**: height offset in object frame
+ **dr**: radial offset in robot frame
+ **t**: [x,y,z] is the initial position of the end-effector in robot frame
+ **nz**: normal versor along which dh is applied (default [0 0 1])

Output arguments:
+ **dt**: [dx,dy,dz] is the position offset of the end-effector in robot frame
</details>

<!-- robot_fsm_interface matlab function -->
<details>
    <summary>
        robot_fsm_interface
    </summary>

High level interface with the robot FSM on Arduino.

    robot_fsm_interface(port, baud, cam, vision_args, trajectory_planning_args, fn_cam2robot_coords, fn_robot_input)

Input arguments:
+ **port**: port of the Arduino serial connection, cf. serialport(...)
+ **baud**: baud rate of the Arduino serial connection, cf. serialport(...)
+ **cam**: webcam object of the camera, cf. webcam(...)
+ **vision_args**: struct of vision parameters, cf below
+ **trajectory_planning_args**: struct of trajectory planning parameters, cf below
+ **fn_cam2robot_coords**: function to convert points from vision to robot frame 
+ **fn_robot_input**: function to acquire input, cf. input(...) or cmdBuffer

For details regarding vision_args and trajectory_planning_args refers to get_target_from_vision(...) and generate_trajectory(...) respectively.

NOTE: this function requires the MATLAB Support Package for USB Webcams.
</details>

<!-- trajectory2serialdata matlab function -->
<details>
    <summary>
        trajectory2serialdata
    </summary>

Prepare trajectory data to be sent to Arduino.

    data_tx = trajectory2serialdata(trajectory_type, delta_t, trajectory)

Input arguments:
+ **type_trajectory**: type of trajectory, cf. generate_trajectory(...)
    + 'pointwise': trajectory defined point by point
    + 'keypoints': trajectory defined via keypoints to be interpolated
+ **delta_t**: timestep of the trajectory execution
+ **trajectory**: NxQNUM array, points of the trajectory

Output arguments:
+ **data_tx**: trajectory data in the format required by Arduino
</details>

### Classes

<!-- cmdBuffer matlab class -->
<details>
    <summary>
        cmdBuffer
    </summary>

Create a LIFO buffer to automatically provide input commands.

Properties:
+ **buffer**: cell array of input commands.

Methods:
+ **cmdBuffer**: constructor that set the initial buffer.
+ **getCmd**: get the last command and remove it from the buffer. The interface is the same of input(...)
+ **cmd2str**: convert a command to string to be plotted.
</details>

An example of usage can be found in [run_robot_fsm](../src/scripts/run_robot_fsm.m).
