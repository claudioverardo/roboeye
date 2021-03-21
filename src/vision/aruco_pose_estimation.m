function [rois, i_arucos, rois_R, rois_t] = aruco_pose_estimation(img, aruco_markers, aruco_real_sides, K, R_cam, t_cam, varargin)
% ARUCO_POSE_ESTIMATION Build Aruco pose estimation pipeline. It executes in
% order aruco_detection(...), roi_pose_estimation(...)
%
%   [rois, i_arucos, rois_R, rois_t] = ARUCO_POSE_ESTIMATION(img, aruco_markers,
%   aruco_real_sides, K, R_cam, t_cam)
%
%   Input arguments:
%   ------------------
%   img:                input image
%   aruco_markers:      input marker dictionary
%   aruco_real_sides:   lengths of the markers in the dictionary [cm]
%   K:                  intrisics matrix of the camera (Matlab convention)
%   R_cam:              rotation matrix of the camera pose in the world frame
%                       (Matlab convention)
%   t_cam:              translation matrix of the camera pose in the world frame
%                       (Matlab convention)
%
%   Parameters:
%   ------------------
%   Refers to aruco_detection(...), roi_pose_estimation(...)
%
%   Output arguments:
%   ------------------
%   rois:               rois matched with the markers
%   i_arucos:           indices of the matched marker for every rois matched 
%   rois_R:             rotation matrices of the roi poses (Matlab convention)
%   rois_t:             translation vectors of the roi poses (Matlab convention)
%
%   See also ARUCO_DETECTION, ROI_POSE_ESTIMATION

    % Default values of parameters
    default_roi_pose_estimation_verbose = 1;
    default_aruco_detection_verbose = 1;
    default_verbose = 1;

    % Input parser
    p = inputParser;
    p.KeepUnmatched = true;
    addParameter(p, 'roi_pose_estimation_verbose', default_roi_pose_estimation_verbose);
    addParameter(p, 'aruco_detection_verbose', default_aruco_detection_verbose);
    addParameter(p, 'verbose', default_verbose);
    parse(p, varargin{:});
    
    % Parse function parameters
    ROI_POSE_ESTIMATION_VERBOSE = p.Results.roi_pose_estimation_verbose;
    ARUCO_DETECTION_VERBOSE = p.Results.aruco_detection_verbose;
    VERBOSE = p.Results.verbose;
    
    % Launch Aruco Detection
    aruco_detection_parameters = p.Unmatched;
    aruco_detection_parameters.verbose = ARUCO_DETECTION_VERBOSE;
    [rois, i_arucos] = aruco_detection(img, aruco_markers, aruco_detection_parameters);
    
    fprintf('----- Aruco Pose Estimation -----\n');

    % Compute pose of matched ROIs in the camera frame
    fprintf('roi_pose_estimation...\n');
    [rois_R, rois_t] = roi_pose_estimation(img, rois, i_arucos, aruco_real_sides, K, R_cam, t_cam, 'verbose', ROI_POSE_ESTIMATION_VERBOSE);

    if VERBOSE > 0
        % TODO: plot of elapsed time at each step
    else

end