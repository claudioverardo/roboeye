function [rois, i_arucos, rois_R, rois_t] = aruco_pose_estimation(img, aruco_markers, aruco_real_side, K_obj, varargin)
    
    % NB: points and K with Matlab convention

    % Default values of parameters
    default_roi_pnp_verbose = 1;
    default_aruco_detection_verbose = 1;
    default_verbose = 1;

    % Input parser
    p = inputParser;
    p.KeepUnmatched = true;
    addParameter(p, 'roi_pnp_verbose', default_roi_pnp_verbose);
    addParameter(p, 'aruco_detection_verbose', default_aruco_detection_verbose);
    addParameter(p, 'verbose', default_verbose);
    parse(p, varargin{:});
    
    % Parse function parameters
    ROI_PNP_VERBOSE = p.Results.roi_pnp_verbose;
    ARUCO_DETECTION_VERBOSE = p.Results.aruco_detection_verbose;
    VERBOSE = p.Results.verbose;
    
    % Launch Aruco Detection
    aruco_detection_parameters = p.Unmatched;
    aruco_detection_parameters.verbose = ARUCO_DETECTION_VERBOSE;
    [rois, i_arucos] = aruco_detection(img, aruco_markers, aruco_detection_parameters);
    
    fprintf('----- Aruco Pose Estimation -----\n');

    % Compute pose of matched ROIs in the camera frame
    fprintf('roi_pnp...\n');
    [rois_R, rois_t] = roi_pnp(img, rois, aruco_real_side, K_obj, ROI_PNP_VERBOSE);

    if VERBOSE > 0
        % TODO: plot of elapsed time at each step
    else

end