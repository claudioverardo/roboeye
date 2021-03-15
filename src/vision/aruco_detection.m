function [rois_matched, i_arucos, k_rots] = aruco_detection(img, aruco_markers, varargin)

    % Default values of parameters
    default_adaptth_sensitivity = 'mean';
    default_adaptth_statistic = 1;
    default_roi_extraction_method = 'adaptth-moore';
    default_canny_th_low = 0.01;
    default_canny_th_high = 0.10;
    default_roi_refinement_method = 'rdp';
    default_rdp_th = 0.1;
    default_roi_sum_angles_tol = 10;
    default_roi_parallelism_tol = 15;
    default_roi_side_th_low = 10;
    default_roi_side_th_high = 700;
    default_roi_bb_padding = 2;
    default_roi_h_side = 40;
    default_roi_hamming_th = 0;
    default_roi_extraction_verbose = 1;
    default_roi_refinement_verbose = 1;
    default_roi_matching_verbose   = 1;
    default_verbose = 1;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'adaptth_sensitivity', default_adaptth_sensitivity);
    addParameter(p, 'adaptth_statistic', default_adaptth_statistic);
    addParameter(p, 'roi_extraction_method', default_roi_extraction_method);
    addParameter(p, 'canny_th_low', default_canny_th_low);
    addParameter(p, 'canny_th_high', default_canny_th_high);
    addParameter(p, 'roi_refinement_method', default_roi_refinement_method);
    addParameter(p, 'rdp_th', default_rdp_th);
    addParameter(p, 'roi_sum_angles_tol', default_roi_sum_angles_tol);
    addParameter(p, 'roi_parallelism_tol', default_roi_parallelism_tol);
    addParameter(p, 'roi_side_th_low', default_roi_side_th_low);
    addParameter(p, 'roi_side_th_high', default_roi_side_th_high);
    addParameter(p, 'roi_bb_padding', default_roi_bb_padding);
    addParameter(p, 'roi_h_side', default_roi_h_side);
    addParameter(p, 'roi_hamming_th', default_roi_hamming_th);
    addParameter(p, 'roi_extraction_verbose', default_roi_extraction_verbose);
    addParameter(p, 'roi_refinement_verbose', default_roi_refinement_verbose);
    addParameter(p, 'roi_matching_verbose', default_roi_matching_verbose);
    addParameter(p, 'verbose', default_verbose);
    parse(p, varargin{:});
    
    % Parse function parameters
    ADAPTTH_SENSITIVITY = p.Results.adaptth_sensitivity;
    ADAPTTH_STATISTIC = p.Results.adaptth_statistic;
    ROI_EXTRACTION_METHOD = p.Results.roi_extraction_method;
    CANNY_TH_LOW = p.Results.canny_th_low;
    CANNY_TH_HIGH = p.Results.canny_th_high;
    ROI_REFINEMENT_METHOD = p.Results.roi_refinement_method;
    RDP_TH = p.Results.rdp_th;
    ROI_SUM_ANGLES_TOL = p.Results.roi_sum_angles_tol;
    ROI_PARALLELISM_TOL = p.Results.roi_parallelism_tol;
    ROI_SIDE_TH_LOW = p.Results.roi_side_th_low; 
    ROI_SIDE_TH_HIGH = p.Results.roi_side_th_high; 
    ROI_BB_PADDING = p.Results.roi_bb_padding; 
    ROI_H_SIDE = p.Results.roi_h_side;
    ROI_HAMMING_TH = p.Results.roi_hamming_th;
    ROI_EXTRACTION_VERBOSE = p.Results.roi_extraction_verbose;
    ROI_REFINEMENT_VERBOSE = p.Results.roi_refinement_verbose;
    ROI_MATCHING_VERBOSE = p.Results.roi_matching_verbose;
    VERBOSE = p.Results.verbose;
    
    fprintf('-------- Aruco Detection --------\n');

    % Conveter image to grayscale
    img_gray = rgb2gray(img);
    
    % Conveter image to binary
    % sensitivity -> [0,1]
    % statistic -> mean, gaussian, median
    img_th = adaptthresh(img_gray, ADAPTTH_SENSITIVITY, 'Statistic', ADAPTTH_STATISTIC); 
    img_bw = imbinarize(img_gray, img_th);
    % figure; imshow(img_bw); title('Binarized image');
    
    % Extract ROIs from image
    fprintf('roi_extraction...\n');
    rois_raw = roi_extraction( ...
        img, img_bw, img_gray, ... 
        'method', ROI_EXTRACTION_METHOD, ...
        'canny_th_low', CANNY_TH_LOW, ...
        'canny_th_high', CANNY_TH_HIGH, ...
        'verbose', ROI_EXTRACTION_VERBOSE ...
    );

    % Select only valid ROIs
    fprintf('roi_refinement...\n');
    [rois_refined, i_rois_refined] = roi_refinement( ...
        img, rois_raw, ... 
        'method', ROI_REFINEMENT_METHOD, ...
        'rdp_th', RDP_TH, ...
        'roi_sum_angles_tol', ROI_SUM_ANGLES_TOL, ...
        'roi_parallelism_tol', ROI_PARALLELISM_TOL, ...
        'roi_side_th_low', ROI_SIDE_TH_LOW, ...
        'roi_side_th_high', ROI_SIDE_TH_HIGH, ...
        'verbose', ROI_REFINEMENT_VERBOSE ...
    );

    % Match ROIs with Arucos
    % NB: rois_matched contains sorted vertices of ROIs
    fprintf('roi_matching...\n');
    [rois_matched, i_rois_matched, i_arucos, k_rots] = roi_matching(...
        img, img_bw, rois_refined, aruco_markers, ...
        'roi_bb_padding', ROI_BB_PADDING, ...
        'roi_h_side', ROI_H_SIDE, ...
        'roi_hamming_th', ROI_HAMMING_TH, ...
        'verbose', ROI_MATCHING_VERBOSE ...
    );

    if VERBOSE > 0
        % TODO: plot of elapsed time at each step
    else

end