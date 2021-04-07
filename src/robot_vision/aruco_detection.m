function [rois_matched, i_arucos, stats] = aruco_detection(img, aruco_markers, varargin)
% ARUCO_DETECTION Build the Aruco detection pipeline. It executes in order
% the functions roi_extraction(...), roi_refinement(...), roi_matching(...).
%
%   [rois_matched, i_arucos] = ARUCO_DETECTION(img, aruco_markers)
%
%   Input arguments:
%   ------------------
%   img:            input image
%   aruco_markers:  markers to be matched
%
%   Parameters:
%   ------------------
%   'verbose':      verbose level of the function (0, 1)
%                   - 0: show nothing
%                   - 1: show log in the command window
%
%   Refer to roi_extraction(...), roi_refinement(...), roi_matching(...) 
%   for details about further allowed parameters.
%
%   Output arguments:
%   ------------------
%   rois_matched:   ROIs matched with the markers
%   i_arucos:       indices of the markers matched with the rois_matched
%   stats:          struct with some performance statistics
%                   - number of ROIs extracted/refined
%                   - times of ROIs extraction/refinement/matching
%
%   See also ROI_EXTRACTION, ROI_REFINEMENT, ROI_MATCHING

    % Conveter image to grayscale
    img_gray = rgb2gray(img);

    % Default values of parameters
    default_roi_extraction_method = 'adaptth-moore';
    default_adaptth_sensitivity = 1;
    default_adaptth_statistic = 'gaussian';
    default_adaptth_neighborhood = 2*floor(size(img_gray)/16)+1;
    default_canny_th_low = 0.01;
    default_canny_th_high = 0.10;
    default_roi_refinement_method = 'rdp';
    default_roi_size_th = 20;
    default_rdp_th = 0.1;
    default_roi_sum_angles_tol = 10;
    default_roi_parallelism_tol = 15;
    default_roi_side_th_low = 1/100;
    default_roi_side_th_high = 1/5;
    default_roi_angle_th_low = 20;
    default_roi_angle_th_high = 160;
    default_roi_bb_padding = 2;
    default_roi_h_side = 40;
    default_roi_hamming_th = 0;
    default_roi_extraction_verbose = 1;
    default_roi_refinement_verbose = 1;
    default_roi_matching_verbose   = 1;
    default_verbose = 1;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'roi_extraction_method', default_roi_extraction_method);
    addParameter(p, 'adaptth_sensitivity', default_adaptth_sensitivity);
    addParameter(p, 'adaptth_statistic', default_adaptth_statistic);
    addParameter(p, 'adaptth_neighborhood', default_adaptth_neighborhood);
    addParameter(p, 'canny_th_low', default_canny_th_low);
    addParameter(p, 'canny_th_high', default_canny_th_high);
    addParameter(p, 'roi_refinement_method', default_roi_refinement_method);
    addParameter(p, 'roi_size_th', default_roi_size_th);
    addParameter(p, 'rdp_th', default_rdp_th);
    addParameter(p, 'roi_sum_angles_tol', default_roi_sum_angles_tol);
    addParameter(p, 'roi_parallelism_tol', default_roi_parallelism_tol);
    addParameter(p, 'roi_side_th_low', default_roi_side_th_low);
    addParameter(p, 'roi_side_th_high', default_roi_side_th_high);
    addParameter(p, 'roi_angle_th_low', default_roi_angle_th_low);
    addParameter(p, 'roi_angle_th_high', default_roi_angle_th_high);
    addParameter(p, 'roi_bb_padding', default_roi_bb_padding);
    addParameter(p, 'roi_h_side', default_roi_h_side);
    addParameter(p, 'roi_hamming_th', default_roi_hamming_th);
    addParameter(p, 'roi_extraction_verbose', default_roi_extraction_verbose);
    addParameter(p, 'roi_refinement_verbose', default_roi_refinement_verbose);
    addParameter(p, 'roi_matching_verbose', default_roi_matching_verbose);
    addParameter(p, 'verbose', default_verbose);
    parse(p, varargin{:});
    
    % Parse function parameters
    ROI_EXTRACTION_METHOD = p.Results.roi_extraction_method;
    ADAPTTH_SENSITIVITY = p.Results.adaptth_sensitivity;
    ADAPTTH_STATISTIC = p.Results.adaptth_statistic;
    ADAPTTH_NEIGHBORHOOD = p.Results.adaptth_neighborhood;
    CANNY_TH_LOW = p.Results.canny_th_low;
    CANNY_TH_HIGH = p.Results.canny_th_high;
    ROI_REFINEMENT_METHOD = p.Results.roi_refinement_method;
    ROI_SIZE_TH = p.Results.roi_size_th;
    RDP_TH = p.Results.rdp_th;
    ROI_SUM_ANGLES_TOL = p.Results.roi_sum_angles_tol;
    ROI_PARALLELISM_TOL = p.Results.roi_parallelism_tol;
    ROI_SIDE_TH_LOW = p.Results.roi_side_th_low; 
    ROI_SIDE_TH_HIGH = p.Results.roi_side_th_high; 
    ROI_ANGLE_TH_LOW = p.Results.roi_angle_th_low;
    ROI_ANGLE_TH_HIGH = p.Results.roi_angle_th_high;
    ROI_BB_PADDING = p.Results.roi_bb_padding; 
    ROI_H_SIDE = p.Results.roi_h_side;
    ROI_HAMMING_TH = p.Results.roi_hamming_th;
    ROI_EXTRACTION_VERBOSE = p.Results.roi_extraction_verbose;
    ROI_REFINEMENT_VERBOSE = p.Results.roi_refinement_verbose;
    ROI_MATCHING_VERBOSE = p.Results.roi_matching_verbose;
    VERBOSE = p.Results.verbose;
    
    if VERBOSE > 0
        fprintf('\n-------- Aruco Detection --------\n');
    end

    % Extract ROIs from input image
    if VERBOSE > 0
        fprintf('roi_extraction...\n');
    end
    [rois_extracted, time_roi_extraction] = roi_extraction( ...
        img, img_gray, ... 
        'method', ROI_EXTRACTION_METHOD, ...
        'adaptth_sensitivity', ADAPTTH_SENSITIVITY, ...
        'adaptth_statistic', ADAPTTH_STATISTIC, ...
        'adaptth_neighborhood', ADAPTTH_NEIGHBORHOOD, ...
        'canny_th_low', CANNY_TH_LOW, ...
        'canny_th_high', CANNY_TH_HIGH, ...
        'verbose', ROI_EXTRACTION_VERBOSE ...
    );

    if VERBOSE > 1
        fprintf('  #rois_raw: %d\n  time: %f s\n', size(rois_extracted,1), time_roi_extraction);
    end

    % Select candidate ROIs for matching
    if VERBOSE > 0
        fprintf('roi_refinement...\n');
    end
    [rois_refined, i_rois_refined, time_roi_refinement] = roi_refinement( ...
        img, rois_extracted, ... 
        'method', ROI_REFINEMENT_METHOD, ...
        'roi_size_th', ROI_SIZE_TH, ...
        'rdp_th', RDP_TH, ...
        'roi_sum_angles_tol', ROI_SUM_ANGLES_TOL, ...
        'roi_parallelism_tol', ROI_PARALLELISM_TOL, ...
        'roi_side_th_low', ROI_SIDE_TH_LOW, ...
        'roi_side_th_high', ROI_SIDE_TH_HIGH, ...
        'roi_angle_th_low', ROI_ANGLE_TH_LOW, ...
        'roi_angle_th_high', ROI_ANGLE_TH_HIGH, ...
        'verbose', ROI_REFINEMENT_VERBOSE ...
    );

    if VERBOSE > 1
        fprintf('  #rois_refined: %d\n  time: %f s\n', size(rois_refined,1), time_roi_refinement);
    end

    % Match Aruco markers with candidate ROIs.
    % NB: rois_matched contains sorted vertices of ROIs
    if VERBOSE > 0
        fprintf('roi_matching...\n');
    end
    [rois_matched, i_rois_matched, i_arucos, time_roi_matching] = roi_matching(...
        img, img_gray, rois_refined, i_rois_refined, aruco_markers, ...
        'roi_bb_padding', ROI_BB_PADDING, ...
        'roi_h_side', ROI_H_SIDE, ...
        'roi_hamming_th', ROI_HAMMING_TH, ...
        'verbose', ROI_MATCHING_VERBOSE ...
    );

    if VERBOSE > 1
        n_rois_matched = size(rois_matched,1);
        fprintf('  #rois_matched: %d\n  time: %f s\n', n_rois_matched, time_roi_matching);
        for i=1:n_rois_matched
            fprintf('  ROI %d -> Aruco %d detected\n', i, i_arucos(i));
        end
    end
    
    % Some performance statistics about aruco_detection
    if nargout > 2
        stats.n_rois_extracted = size(rois_extracted,1);
        stats.n_rois_refined = size(rois_refined,1);
        stats.time_roi_extraction = time_roi_extraction;
        stats.time_roi_refinement = time_roi_refinement;
        stats.time_roi_matching = time_roi_matching;
    end

end