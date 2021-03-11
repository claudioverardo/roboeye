function [rois_found, i_rois, i_arucos, k_rots] = aruco_detection(img, aruco_markers, varargin)

    % Default values of parameters
    default_canny_th_low = 0.01;
    default_canny_th_high = 0.10;
    default_rdp_th = 0.1;
    default_roi_size_th_low = 10*10;
    default_roi_size_th_high = 700*700;
    default_sum_angles_tolerance  = 10;
    default_parallelism_tolerance = 15;
    default_bb_padding = 2;
    default_roi_th_size = 0.5;
    default_roi_h_side = 40;
    default_hamming_th = 0;
    
    default_extract_roi_verbose = 1;
    default_match_roi_verbose   = 1;
    default_verbose = 1;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'canny_th_low', default_canny_th_low);
    addParameter(p, 'canny_th_high', default_canny_th_high);
    addParameter(p, 'rdp_th', default_rdp_th);
    addParameter(p, 'roi_size_th_low', default_roi_size_th_low);
    addParameter(p, 'roi_size_th_high', default_roi_size_th_high);
    addParameter(p, 'sum_angles_tolerance', default_sum_angles_tolerance);
    addParameter(p, 'parallelism_tolerance', default_parallelism_tolerance);
    addParameter(p, 'bb_padding', default_bb_padding);
    addParameter(p, 'roi_th_size', default_roi_th_size);
    addParameter(p, 'roi_h_side', default_roi_h_side);
    addParameter(p, 'hamming_th', default_hamming_th);
    addParameter(p, 'extract_roi_verbose', default_extract_roi_verbose);
    addParameter(p, 'match_roi_verbose', default_match_roi_verbose);
    addParameter(p, 'verbose', default_verbose);
    parse(p, varargin{:});
    
    % Parse function parameters
    CANNY_TH_LOW = p.Results.canny_th_low;
    CANNY_TH_HIGH = p.Results.canny_th_high;
    RDP_TH = p.Results.rdp_th;
    ROI_SIZE_TH_LOW = p.Results.roi_size_th_low; 
    ROI_SIZE_TH_HIGH = p.Results.roi_size_th_high; 
    SUM_ANGLES_TOLERANCE = p.Results.sum_angles_tolerance;
    PARALLELISM_TOLERANCE = p.Results.parallelism_tolerance;
    BB_PADDING = p.Results.bb_padding; 
    ROI_H_SIDE = p.Results.roi_h_side;
    HAMMING_TH = p.Results.hamming_th;
    EXTRACT_ROI_VERBOSE = p.Results.extract_roi_verbose;
    MATCH_ROI_VERBOSE = p.Results.match_roi_verbose;
    VERBOSE = p.Results.verbose;

    % Conveter image to grayscale
    img_gray = rgb2gray(img);

    % Conveter image to binary
    img_th = adaptthresh(img_gray, 1, 'Statistic', 'gaussian'); % mean, gaussian, median
    img_bw = imbinarize(img_gray, img_th);
    
    % Plot binarization output
    if VERBOSE > 1
        figure;
        imshow(img_bw);
        title('Binarized image');
    end

    % Canny edge detector
    % img_canny = edge(img_gray, 'canny', [CANNY_TH_LOW, CANNY_TH_HIGH]);
    
    % Plot Canny output
    % if VERBOSE > 1
    %    figure;
    %    imshow(img_canny);
    %    title('Canny Edge Detector');
    % end

    % Extract morphological components - DFS [C-implementation]
    % [components, tails] = roi_extraction_c(img_canny, size(img_canny, 1), size(img_canny, 2));
    % rois_raw = components;

    % Extract morphological components - DFS [MATLAB-implementation]
    % components = roi_extraction(img_canny, EXTRACT_ROI_VERBOSE);
    % rois_raw = components(:,1);
    
    % Extract morphological components - Moore-Neighbor tracing
    rois_raw = bwboundaries(1-img_bw,'noholes');
    for k=1:size(rois_raw,1)
        rois_raw{k} = circshift(rois_raw{k},1,2);
    end
    
    % Plot extracted ROIs
    if VERBOSE > 1
        figure;
        imshow(img);
        for k=1:size(rois_raw,1)
           hold on;
           line([rois_raw{k,1}(:,1); rois_raw{k,1}(1,1)], ...
                [rois_raw{k,1}(:,2); rois_raw{k,1}(1,2)], ...
                'color','r','linestyle','-','linewidth',1.5, ...
                'marker','o','markersize',5);
        end
        title(sprintf('Extracted ROIs N=%d', size(rois_raw,1)));
    end
    
    % Select only valid ROIs
    rois = roi_refinement( ...
        rois_raw, ... 
        'rdp_th', RDP_TH, ...
        'roi_size_th_low', ROI_SIZE_TH_LOW, ...
        'roi_size_th_high', ROI_SIZE_TH_HIGH, ...
        'sum_angles_tolerance', SUM_ANGLES_TOLERANCE, ...
        'parallelism_tolerance', PARALLELISM_TOLERANCE ...
    );
    
    % Plot refined ROIs
    if VERBOSE > 0
        figure;
        imshow(img);
        for k=1:size(rois,1)
           hold on;
           line([rois{k,1}(:,1); rois{k,1}(1,1)], ...
                [rois{k,1}(:,2); rois{k,1}(1,2)], ...
                'color','g','linestyle','-','linewidth',1.5, ...
                'marker','o','markersize',5);
        end
        title(sprintf('Refined ROIs N=%d', size(rois,1)));
    end
    
    % Launch Aruco matching
    % NB: rois_found contains sorted vertices of ROIs
    [rois_found, i_rois, i_arucos, k_rots] = roi_matching(...
        img, img_bw, rois, aruco_markers, ...
        'bb_padding', BB_PADDING, ...
        'roi_h_side', ROI_H_SIDE, ...
        'hamming_th', HAMMING_TH, ...
        'verbose', MATCH_ROI_VERBOSE ...
    );

    % Plot matched ROIs
    if VERBOSE > 0
        figure;
        imshow(img);
        for k=1:size(rois_found,1)
           hold on;
           line([rois_found{k,1}(:,1); rois_found{k,1}(1,1)], ...
                [rois_found{k,1}(:,2); rois_found{k,1}(1,2)], ...
                'color','r','linestyle','-','linewidth',1.5, ...
                'marker','o','markersize',5);
        end
        title(sprintf('Matched ROIs N=%d', size(rois_found,1)));
    end

end