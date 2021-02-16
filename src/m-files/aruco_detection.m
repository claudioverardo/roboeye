function [roi, i_arucos, k_rots] = aruco_detection(img, aruco_markers, varargin)

    % Default values of parameters
    default_canny_th_low  = 0.01;
    default_canny_th_high = 0.10;
    default_rdp_th        = 0.1;
    default_bb_padding    = 2;
    default_roi_th_size   = 0.5;
    default_roi_h_side    = 40;
    default_hamming_th    = 0;
    
    default_extract_roi_verbose = 1;
    default_match_roi_verbose   = 1;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'canny_th_low', default_canny_th_low);
    addParameter(p, 'canny_th_high', default_canny_th_high);
    addParameter(p, 'rdp_th', default_rdp_th);
    addParameter(p, 'bb_padding', default_bb_padding);
    addParameter(p, 'roi_th_size', default_roi_th_size);
    addParameter(p, 'roi_h_side', default_roi_h_side);
    addParameter(p, 'hamming_th', default_hamming_th);
    addParameter(p, 'extract_roi_verbose', default_extract_roi_verbose);
    addParameter(p, 'match_roi_verbose', default_match_roi_verbose);
    parse(p, varargin{:});
    
    % Parse function parameters
    CANNY_TH_LOW  = p.Results.canny_th_low;
    CANNY_TH_HIGH = p.Results.canny_th_high;
    RDP_TH        = p.Results.rdp_th;
    BB_PADDING    = p.Results.bb_padding; 
    ROI_TH_SIZE   = p.Results.roi_th_size; 
    ROI_H_SIDE    = p.Results.roi_h_side;
    HAMMING_TH    = p.Results.hamming_th;
    EXTRACT_ROI_VERBOSE = p.Results.extract_roi_verbose;
    MATCH_ROI_VERBOSE   = p.Results.match_roi_verbose;
    
    % Area of the image
    img_size = size(img,1)*size(img,2);

    % Conveter image to grayscale
    img_gray = rgb2gray(img);

    % Conveter image to binary
    img_bw = imbinarize(img_gray);

    % Markers
    n_aruco_markers = size(aruco_markers,3);

    % Edge detection image
    img_canny = edge(img_gray, 'canny', [CANNY_TH_LOW, CANNY_TH_HIGH]);

    % Extract morphological components
    %components = roi_extraction(EXTRACT_ROI_VERBOSE);
    [components, tails] = roi_extraction_c(img_canny, size(img_canny, 1), size(img_canny, 2));
    rois_raw = components;
    
    % Select only valid ROIs
    rois = roi_refinement(rois_raw, RDP_TH);
         
    % Launch Aruco matching
    % verbose = 2;
    % rois = components(1:8, 3);
    % rois = components(:, 3);
    [i_rois, i_arucos, k_rots] = roi_matching(...
        img, rois, aruco_markers, ...
        'bb_padding', BB_PADDING, ...
        'roi_th_size', ROI_TH_SIZE, ...
        'roi_h_side', ROI_H_SIDE, ...
        'hamming_th', HAMMING_TH, ...
        'verbose', MATCH_ROI_VERBOSE ...
    );

    roi = rois{i_rois};

end