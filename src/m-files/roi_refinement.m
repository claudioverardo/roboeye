function [rois_refined, i_rois_refined] = roi_refinement(rois_raw, varargin)
    % rois_refined:   valid rois among the rois_raw
    % i_rois_refined: indices of the rois_refined in the rois_raw cell array

    % Default values of parameters
    default_method = 'rdp';
    default_rdp_th = 0.1;
    default_roi_side_th_low = 10;
    default_roi_side_th_high = 700;
    default_roi_sum_angles_tol = 10;
    default_roi_parallelism_tol = 15;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'method', default_method);
    addParameter(p, 'rdp_th', default_rdp_th);
    addParameter(p, 'roi_sum_angles_tol', default_roi_sum_angles_tol);
    addParameter(p, 'roi_parallelism_tol', default_roi_parallelism_tol);
    addParameter(p, 'roi_side_th_low', default_roi_side_th_low);
    addParameter(p, 'roi_side_th_high', default_roi_side_th_high);
    parse(p, varargin{:});
    
    % Parse function parameters
    METHOD = p.Results.method;
    RDP_TH = p.Results.rdp_th;
    ROI_SUM_ANGLES_TOL = p.Results.roi_sum_angles_tol;
    ROI_PARALLELISM_TOL = p.Results.roi_parallelism_tol;
    ROI_SIDE_TH_LOW = p.Results.roi_side_th_low; 
    ROI_SIDE_TH_HIGH = p.Results.roi_side_th_high; 
    
    if strcmp(METHOD, 'rdp')
        % Ramer–Douglas–Peucker refinement
        roi_refinement_core = @(roi_raw) roi_refinement_rdp(roi_raw, RDP_TH);
    elseif strcmp(METHOD, 'geometric')
        % Geometric refinement
        roi_refinement_core = @(roi_raw) roi_refinement_geometric(roi_raw);
    else
        % Invalid ROI_REFINEMENT_METHOD
        error('Error: Invalid ROI_REFINEMENT_METHOD = \"%s\"', METHOD);
    end
    
    rois_refined = cell(0);
    i_rois_refined = [];

    for i = 1:size(rois_raw, 1)
        
        % Refine ROI with the chosen method
        roi_refined = roi_refinement_core(rois_raw{i});
        
        % Check if the ROI is a valid quadrilateral
        if check_quadrilateral(roi_refined, ROI_SUM_ANGLES_TOL, ROI_PARALLELISM_TOL, ROI_SIDE_TH_LOW, ROI_SIDE_TH_HIGH) == 1
                
            % ROI ok, add to rois
            rois_refined{end+1,1} = roi_refined;
            i_rois_refined(end+1) = i;
        
        end
        
    end

end

function roi_refined = roi_refinement_rdp(roi_raw, RDP_TH)
            
    % Calculate polyfit Ramer–Douglas-Pecker Algorithm
    % NOTE: MATLAB >= 2019b
    roi_refined = reducepoly(roi_raw, RDP_TH);  

    % Remove last point (= to the first one)
    roi_refined(end,:) = [];
    
end

function roi_refined = roi_refinement_geometric(roi_raw)

    roi_refined = zeros(4,2);

    % Top-left corner
    idx_t = find(roi_raw(:,2) == min(roi_raw(:,2)));
    idx_tl = find(roi_raw(idx_t, 1) == min(roi_raw(idx_t, 1)));
    roi_refined(1,:) = roi_raw(idx_t(idx_tl(1)),:);

    % Top-right corner
    idx_r = find(roi_raw(:,1) == max(roi_raw(:,1)));
    idx_tr = find(roi_raw(idx_r, 2) == min(roi_raw(idx_r, 2)));
    roi_refined(2,:) = roi_raw(idx_r(idx_tr(1)),:);

    % Bottom-right corner
    idx_b = find(roi_raw(:,2) == max(roi_raw(:,2)));
    idx_br = find(roi_raw(idx_b, 1) == max(roi_raw(idx_b, 1)));
    roi_refined(3,:) = roi_raw(idx_b(idx_br(1)),:);

    % Bottom-left corner 
    idx_l = find(roi_raw(:,1) == min(roi_raw(:,1)));
    idx_bl = find(roi_raw(idx_l, 2) == max(roi_raw(idx_l, 2)));
    roi_refined(4,:) = roi_raw(idx_l(idx_bl(1)),:);

end