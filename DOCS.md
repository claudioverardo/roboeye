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
+ **ruco_markers**: input marker dictionary
+ **varargin**: collection of optional parameters check official Matlab documentation

Output params:
+ **rois_matched**: matched rois among the rois
+ **i_arucos**: indices of the matched marker for every rois matched
</details>

<!-- aruco_pose_estimation matlab function -->
<details>
    <summary>
        aruco_pose_estimation
    </summary>

    [rois, i_arucos, rois_R, rois_t] = aruco_pose_estimation(img, aruco_markers, aruco_real_sides, K, R_cam, t_cam, varargin)

Input params:
+ **img**:                input image
+ **aruco_markers**:      input marker dictionary
+ **aruco_real_sides**:   lengths of the markers in the dictionary [cm]
+ **K**:                  intrisics matrix of the camera (Matlab convention)
TODO

Output params:
+ **rois**:               rois matched with the markers
+ **i_arucos**:           indices of the matched marker for every rois matched 
+ **rois_R**:             rotation matrices of the roi poses (Matlab convention)
+ **rois_t**:             translation vectors of the roi poses (Matlab convention)
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
+ **img_size**:   1x2 (rows img limit, columns img limit)

Output params:
+ **check_ans**:  1 if the point is inside the image size 0 otherwise
</details>

<!-- check_quadrilateral matlab function -->
<details>
    <summary>
        check_quadrilateral
    </summary>

    is_valid_quad = check_quadrilateral(points, varargin)

Input params:
+ **points**:   Array Nx2, example [ [x1,y1]; [x2,y2]; ...; [xN,yN] ]
+ **varargin**: collection of optional parameters check official Matlab documentation

Output params:
+ **is_valid_quad**: return 1 if this is a valid quadrilateral 0 otherwise
</details>

<!-- homography matlab function -->
<details>
    <summary>
        homography
    </summary>

    Y = homography(X, H)

Input params:
+ **X**:      Input set points (inhomogeneous coordinates)
+ **H**:      Linear transformation between homogeneous coordinates (Matlab convention: hom(Y) = hom(X)*H)

Output params:
+ **Y**:      Transformed output points (Matlab convention)
</details>

<!-- pnp_lin matlab function -->
<details>
    <summary>
        pnp_lin
    </summary>

    [R, t, reproj_err] = pnp_lin(X_image, X_world, K)

Input params:
+ **X_image**:    Nx2 array, 2D image points
+ **X_world**:    Nx3 array, 3D world points

Output params:
+ **R**:          rotation matrix 3x3 (Matlab convention)
+ **t**:          translate vector 1x3 (Matlab convention)
+ **reproj_err**: reprojection error (RMS value)
</details>

<!-- pnp_nonlin matlab function -->
<details>
    <summary>
        pnp_nonlin
    </summary>

    [R, t] = PNP_NONLIN(R0, t0, X_image, X_world, K) refines the input camera pose R0, t0 from a set of 2D-3D correspondences defined by X_image, X_world 
%   respectively. The algorithm minimizes the reprojection errors.

    [R, t, reproj_err] = PNP_NONLIN(R0, t0, X_image, X_world, K) return also the RMS value of the reprojection errors of the 3D-2D correspondences.

Input params:
+ **R0**:         Initial rotation matrix for the non-linear iterative method, typically calculate through the pnp_lin function
+ **t0**:         Initial translate vector for the non-linear iterative method, typically calculate through the pnp_lin function
+ **X_image**:    Nx2 array, 2D image points
+ **X_world**:    Nx3 array, 3D world points
+ **K**:          Intrisics matrix of the input camera

Output params:
+ **R**:          rotation matrix 3x3 (Matlab convetion)
+ **t**:          translate vector 1x3 (Matlab convetion)
+ **reproj_err**: reprojection error (RMS value)
</details>

<!-- reprojection_error matlab function -->
<details>
    <summary>
        reprojection_error
    </summary>

    [err, J_ext] = reprojection_error(m, M, K, R, t)

Input params:
+ **m**:      2D point (image point)
+ **M**:      3D point (world point)
+ **K**:      Intrisics matrix of the input camera
+ **R**:      Rotation matrix 3x3 (Matlab convention)
+ **t**:      Translation vector 1x3 (Matlab convention)

Output params:
+ **err**:    component-wise reprojection error between m and M*[R; t]*K
+ **J_ext**:  Jacobian of err wrt the external parameters
</details>

<!-- roi_extraction matlab function -->
<details>
    <summary>
        roi_extraction
    </summary>

    rois_raw = roi_extraction(img, img_gray, varargin)

Input params:
+ **img**:      input image
+ **img_gray**: input image grayscale
+ **varargin**: collection of optional parameters check official Matlab

Output params:
+ **rois_raw**: extracted rois without any refinement
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

    [rois_matched, i_rois_matched, i_arucos] = roi_matching(img, img_gray, rois, aruco_markers, varargin)

Input params:
+ **img**: input image
+ **img_gray**: input image grayscale
+ **rois**: region of interest candidated for matching with markers
+ **aruco_markers**: input marker dictionary
+ **varargin**: collection of optional parameters check official Matlab

Output params:
+ **rois_matched**: matched rois among the rois
+ **i_rois_matched**: indices of the rois_matched in the rois cell array
+ **i_arucos**: indices of the matched marker for every rois matched 
</details>

<!-- roi_pose_estimation matlab function -->
<details>
    <summary>
        roi_pose_estimation
    </summary>

TODO
</details>

<!-- roi_refinement matlab function -->
<details>
    <summary>
        roi_refinement
    </summary>

    [rois_refined, i_rois_refined] = roi_refinement(img, rois_raw, varargin)

Input params:
+ **img**: input image
+ **rois_raw**: extracted region of interests
+ **varargin**: collection of optional parameters check official Matlab

Output params:
+ **rois_refined**: valid rois among the rois_raw
+ **i_rois_refined**: indices of the rois_refined in the rois_raw cell array
</details>

<!-- roi_refinement matlab function -->
<details>
    <summary>
        rpy2rot
    </summary>

    [R, J_roll, J_pitch, J_yaw] = rpy2rot(a)

Input params:
+ **a**: a(1) = roll  (x-axis), a(2) = pitch (y-axis), a(3) = yaw (z-axis)

Output params:
+ **R**: R = Rx(roll)*Ry(pitch)*Rz(yaw)
+ **J_roll**: Jacobian of R wrt roll
+ **J_pitch**: Jacobian of R wrt pitch
+ **J_yaw**: Jacobian of R wrt yaw
</details>

<a name="robot-control"></a>
## Robot Control