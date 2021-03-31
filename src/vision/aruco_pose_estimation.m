function [rois, i_arucos, rois_R, rois_t] = aruco_pose_estimation(img, aruco_markers, aruco_real_sides, K, R_cam, t_cam, k, varargin)
% ARUCO_POSE_ESTIMATION Build the Aruco pose estimation pipeline. It executes
% in order the functions aruco_detection(...), roi_pose_estimation(...).
%
%   [rois, i_arucos, rois_R, rois_t] = ARUCO_POSE_ESTIMATION(img, aruco_markers,
%   aruco_real_sides, K, R_cam, t_cam, k)
%
%   Input arguments:
%   ------------------
%   img:                input image
%   aruco_markers:      markers to be matched
%   aruco_real_sides:   real world lengths of the sides of the markers [cm]
%   K:                  intrisics matrix of the camera (Matlab convention)
%   R_cam:              rotation matrix of the camera extrinsics in the
%                       world frame (Matlab convention)
%   t_cam:              translation vector of the camera extrinsics in the 
%                       world frame (Matlab convention)
%   k:                  radial distortion coefficients of the camera
%
%   Parameters:
%   ------------------
%   'verbose':          verbose level of the function (0, 1)
%                       - 0: show nothing
%                       - 1: show log in the command window
%
%   Refer to aruco_detection(...), roi_pose_estimation(...) for details about
%   further allowed parameters.
%
%   Output arguments:
%   ------------------
%   rois:               ROIs matched with the markers
%   i_arucos:           indices of the markers matched with the rois
%   rois_R:             rotation matrices of the roto-translations that map
%                       points from the ROIs frames into the world frame
%                       (Matlab convention)
%   rois_t:             translation vectors of the roto-translations that map
%                       points from the ROIs frames into the world frame 
%                       (Matlab convention)
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
    
    if VERBOSE > 0
        fprintf('\n----- Aruco Pose Estimation -----\n');
    end

    % Compute pose of matched ROIs in the camera frame
    if VERBOSE > 0
        fprintf('roi_pose_estimation...\n');
    end
    [rois_R, rois_t, err_lin, err_nonlin, time_roi_pose_estimation] = roi_pose_estimation( ...
        img, rois, i_arucos, aruco_real_sides, K, R_cam, t_cam, k, ...
        'verbose', ROI_POSE_ESTIMATION_VERBOSE ...
    );

    if VERBOSE > 1
        fprintf('  time: %f s\n', time_roi_pose_estimation);
        for i=1:length(rois)
            fprintf('  ROI %d -> RMS reproj error lin: %f -- nonlin: %f\n', i, err_lin(i), err_nonlin(i));
        end
    end

end