function [rois_matched, i_arucos, k_rots] = aruco_detection(img, aruco_markers, varargin)

    % Default values of parameters
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
    default_roi_matching_verbose   = 1;
    default_verbose = 1;
    
    % Input parser
    p = inputParser;
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
    addParameter(p, 'roi_matching_verbose', default_roi_matching_verbose);
    addParameter(p, 'verbose', default_verbose);
    parse(p, varargin{:});
    
    % Parse function parameters
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
    ROI_MATCHING_VERBOSE = p.Results.roi_matching_verbose;
    VERBOSE = p.Results.verbose;

    % Conveter image to grayscale
    img_gray = rgb2gray(img);
    
    % Conveter image to binary
    img_th = adaptthresh(img_gray, 1, 'Statistic', 'gaussian'); % mean, gaussian, median
    img_bw = imbinarize(img_gray, img_th);
    % img_bw = imbinarize(img_gray);
    
    % Plot binarization output
    if VERBOSE > 1
        figure;
        imshow(img_bw);
        title('Binarized image');
    end
    
    if strcmp(ROI_EXTRACTION_METHOD, 'adaptth-moore')
        
        % Extract morphological components - Moore-Neighbor tracing
        rois_raw = bwboundaries(1-img_bw,'noholes');
        for k=1:size(rois_raw,1)
            % ij --> xy coordinates
            rois_raw{k} = circshift(rois_raw{k},1,2);
            % remove the last point (equal to the first one)
            rois_raw{k}(end,:) = [];
            % NOTE: the output of bwboundaries may contain repeated points (pay attention with roi_refinement_geometric)
        end
        
    elseif strcmp(ROI_EXTRACTION_METHOD, 'canny-dfs')

        % Canny edge detector
        img_canny = edge(img_gray, 'canny', [CANNY_TH_LOW, CANNY_TH_HIGH]);

        % Plot Canny output
        if VERBOSE > 1
           figure;
           imshow(img_canny);
           title('Canny Edge Detector');
        end

        % Extract morphological components - DFS [C-implementation]
        % [components, tails] = roi_extraction_c(img_canny, size(img_canny, 1), size(img_canny, 2));
        % rois_raw = components;

        % Extract morphological components - DFS [MATLAB-implementation]
        components = roi_extraction(img_canny, ROI_EXTRACTION_VERBOSE);
        rois_raw = components(:,1);
    
    else
        
        % Invalid ROI_EXTRACTION_METHOD
        error('Error: Invalid ROI_EXTRACTION_METHOD = \"%s\"', ROI_EXTRACTION_METHOD);
    
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
    [rois_refined, i_rois_refined] = roi_refinement( ...
        rois_raw, ... 
        'method', ROI_REFINEMENT_METHOD, ...
        'rdp_th', RDP_TH, ...
        'roi_sum_angles_tol', ROI_SUM_ANGLES_TOL, ...
        'roi_parallelism_tol', ROI_PARALLELISM_TOL, ...
        'roi_side_th_low', ROI_SIDE_TH_LOW, ...
        'roi_side_th_high', ROI_SIDE_TH_HIGH ...
    );
    
    % Plot refined ROIs
    if VERBOSE > 0
        figure;
        imshow(img);
        for k=1:size(rois_refined,1)
           hold on;
           line([rois_refined{k,1}(:,1); rois_refined{k,1}(1,1)], ...
                [rois_refined{k,1}(:,2); rois_refined{k,1}(1,2)], ...
                'color','g','linestyle','-','linewidth',1.5, ...
                'marker','o','markersize',5);
        end
        title(sprintf('Refined ROIs N=%d', size(rois_refined,1)));
    end
    
    % Launch Aruco matching
    % NB: rois_found contains sorted vertices of ROIs
    [rois_matched, i_rois_matched, i_arucos, k_rots] = roi_matching(...
        img, img_bw, rois_refined, aruco_markers, ...
        'roi_bb_padding', ROI_BB_PADDING, ...
        'roi_h_side', ROI_H_SIDE, ...
        'roi_hamming_th', ROI_HAMMING_TH, ...
        'verbose', ROI_MATCHING_VERBOSE ...
    );

    % Plot matched ROIs
    if VERBOSE > 0
        figure;
        imshow(img);
        for k=1:size(rois_matched,1)
           hold on;
           line([rois_matched{k,1}(:,1); rois_matched{k,1}(1,1)], ...
                [rois_matched{k,1}(:,2); rois_matched{k,1}(1,2)], ...
                'color','r','linestyle','-','linewidth',1.5, ...
                'marker','o','markersize',5);
        end
        title(sprintf('Matched ROIs N=%d', size(rois_matched,1)));
    end

end