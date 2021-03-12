function rois = roi_refinement(rois_raw, varargin)

    % Default values of parameters
    default_rdp_th = 0.1;
    default_roi_size_th_low = 10*10;
    default_roi_size_th_high = 700*700;
    default_sum_angles_tolerance  = 10;
    default_parallelism_tolerance = 15;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'rdp_th', default_rdp_th);
    addParameter(p, 'roi_size_th_low', default_roi_size_th_low);
    addParameter(p, 'roi_size_th_high', default_roi_size_th_high);
    addParameter(p, 'sum_angles_tolerance', default_sum_angles_tolerance);
    addParameter(p, 'parallelism_tolerance', default_parallelism_tolerance);
    parse(p, varargin{:});
    
    % Parse function parameters
    RDP_TH = p.Results.rdp_th;
    ROI_SIZE_TH_LOW = p.Results.roi_size_th_low; 
    ROI_SIZE_TH_HIGH = p.Results.roi_size_th_high; 
    SUM_ANGLES_TOLERANCE = p.Results.sum_angles_tolerance;
    PARALLELISM_TOLERANCE = p.Results.parallelism_tolerance;
    
    rois = cell(0);

    for i = 264:size(rois_raw, 1)
        
        % Calculate polyfit Ramer–Douglas-Pecker Algorithm
        % NOTE: MATLAB >= 2019b
        % roi = reducepoly(rois_raw{i}, RDP_TH);  

        % Remove last point (= to the first one)
        % roi(end,:) = [];
        
        idx_t = find(rois_raw{i}(:,2) == min(rois_raw{i}(:,2)));
        idx_tl = find(rois_raw{i}(idx_t, 1) == min(rois_raw{i}(idx_t, 1)) + idx_t(1) - 1);
        roi(1,:) = rois_raw{i}(idx_tl,:);
        
        idx_r = find(rois_raw{i}(:,1) == max(rois_raw{i}(:,1)));
        idx_tr = find(rois_raw{i}(idx_r, 2) == min(rois_raw{i}(idx_r, 2))) + idx_r(1) - 1;
        roi(2,:) = rois_raw{i}(idx_tr,:);
        
        idx_b = find(rois_raw{i}(:,2) == max(rois_raw{i}(:,2)));
        idx_br = find(rois_raw{i}(idx_b, 1) == max(rois_raw{i}(idx_b, 1))) + idx_b(1) - 1;
        roi(3,:) = rois_raw{i}(idx_br,:);
        
        idx_l = find(rois_raw{i}(:,1) == min(rois_raw{i}(:,1)));
        idx_bl = find(rois_raw{i}(idx_l, 2) == max(rois_raw{i}(idx_l, 2))) + idx_l(1) - 1;
        roi(4,:) = rois_raw{i}(idx_bl,:);
        
        % Check if the ROI is a valid quadrilateral
        if check_quadrilateral(roi, ...
            'sum_angles_tolerance', SUM_ANGLES_TOLERANCE, ...
            'parallelism_tolerance', PARALLELISM_TOLERANCE) == 1
        
            % Find the ROI bounding box
            bb_idx_top    = min(roi(:,2));
            bb_idx_bottom = max(roi(:,2));
            bb_idx_left   = min(roi(:,1));
            bb_idx_right  = max(roi(:,1));
            bb_height     = bb_idx_bottom - bb_idx_top;
            bb_width      = bb_idx_right - bb_idx_left;
            bb_size       = bb_width * bb_height;

            % Check if the ROI boundary box has very small/huge dimensions
            if bb_size > ROI_SIZE_TH_LOW && bb_size < ROI_SIZE_TH_HIGH
                
                rois{size(rois,1)+1,1} = roi;

            end
        
        end
        
    end

end