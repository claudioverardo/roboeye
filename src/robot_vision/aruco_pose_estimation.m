function [rois, i_arucos, rois_R, rois_t, stats] = aruco_pose_estimation(img, aruco_markers, aruco_real_sides, K, R_cam, t_cam, k, varargin)
% ARUCO_POSE_ESTIMATION Build the Aruco pose estimation pipeline. It executes
% in order the functions aruco_detection(...), roi_pose_estimation(...).
%
%   [rois, i_arucos, rois_R, rois_t, stats] = ARUCO_POSE_ESTIMATION(img,
%   aruco_markers, aruco_real_sides, K, R_cam, t_cam, k)
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
%   stats:              struct with some performance statistics
%                       - number of ROIs extracted/refined
%                       - times of ROIs extraction/refinement/matching and
%                         pose estimation    
%                       - reprojection errors of lin/nonlin PnP
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
    [rois, i_arucos, stats_detection] = aruco_detection( ...
        img, aruco_markers, ...
        aruco_detection_parameters ...
    );
    
    if VERBOSE > 0
        fprintf('\n----- Aruco Pose Estimation -----\n');
    end

    % Compute pose of matched ROIs in the camera frame
    if VERBOSE > 0
        fprintf('roi_pose_estimation...\n');
    end
    [rois_R, rois_t, reproj_err_lin, reproj_err_nonlin, time_roi_pose_estimation] = roi_pose_estimation( ...
        img, rois, i_arucos, aruco_real_sides, K, R_cam, t_cam, k, ...
        'verbose', ROI_POSE_ESTIMATION_VERBOSE ...
    );

    if VERBOSE > 1
        fprintf('  time: %f s\n', time_roi_pose_estimation);
        for i=1:length(rois)
            fprintf('  ROI %d -> RMS reproj error lin: %f -- nonlin: %f\n', i, reproj_err_lin(i), reproj_err_nonlin(i));
        end
    end
    
    % Some performance statistics about aruco_pose_estimation
    if nargout > 4
        stats.n_rois_extracted = stats_detection.n_rois_extracted;
        stats.n_rois_refined = stats_detection.n_rois_refined;
        stats.time_roi_extraction = stats_detection.time_roi_extraction;
        stats.time_roi_refinement = stats_detection.time_roi_refinement;
        stats.time_roi_matching = stats_detection.time_roi_matching;
        stats.time_roi_pose_estimation = time_roi_pose_estimation;
        stats.reproj_err_lin = reproj_err_lin;
        stats.reproj_err_nonlin = reproj_err_nonlin;
    end

end