function [rois_refined, i_rois_refined] = roi_refinement(img, rois_raw, varargin)
% ROI_REFINEMENT Select candidate ROIs for matching
%
%   [rois_refined, i_rois_refined] = ROI_REFINEMENT(img, rois_raw)
%
%   Input arguments:
%   ------------------
%   img:                    input image
%   rois_raw:               extracted region of interests
%
%   Parameters:
%   ------------------
%   'method':               Choose the roi refinement algorithm
%                           'rdp': Ramer-Douglas�Peucker 
%                           'geometric': Extreme corners for each quadrilateral side
%   'roi_size_th':          minimun number of points require by each roi to be processed
%   'rdp_th':               threshold of the Ramer-Douglas�Peucker algorithm
%   'roi_sum_angles_tol':   tolerance on the sum of the internal angles [degrees]
%   'roi_parallelism_tol':  tolerance on the angle between opposite sides [degrees]
%   'roi_side_th_low':      lower threshold on the length of each side
%                           (normalized wrt the diagonal of the image)
%   'roi_side_th_high':     higher threshold on the length of each side
%                           (normalized wrt the diagonal of the image)
%   'verbose':              verbose level of the function (allowed values 0, 1, 2)
%
%   Output arguments:
%   ------------------
%   rois_refined:           valid rois among the rois_raw
%   i_rois_refined:         indices of the rois_refined in the rois_raw cell array
%
%   NOTE to use the Ramer-Douglas�Peucker('rdp') you need Matlab >= 2019b
%
%   See also ARUCO_DETECTION

    % Default values of parameters
    default_method = 'rdp';
    default_roi_size_th = 20;
    default_rdp_th = 0.1;
    default_roi_side_th_low = 1/100;
    default_roi_side_th_high = 1/5;
    default_roi_sum_angles_tol = 10;
    default_roi_parallelism_tol = 15;
    default_verbose = 1;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'method', default_method);
    addParameter(p, 'roi_size_th', default_roi_size_th);
    addParameter(p, 'rdp_th', default_rdp_th);
    addParameter(p, 'roi_sum_angles_tol', default_roi_sum_angles_tol);
    addParameter(p, 'roi_parallelism_tol', default_roi_parallelism_tol);
    addParameter(p, 'roi_side_th_low', default_roi_side_th_low);
    addParameter(p, 'roi_side_th_high', default_roi_side_th_high);
    addParameter(p, 'verbose', default_verbose);
    parse(p, varargin{:});
    
    % Parse function parameters
    METHOD = p.Results.method;
    ROI_SIZE_TH = p.Results.roi_size_th;
    RDP_TH = p.Results.rdp_th;
    ROI_SUM_ANGLES_TOL = p.Results.roi_sum_angles_tol;
    ROI_PARALLELISM_TOL = p.Results.roi_parallelism_tol;
    ROI_SIDE_TH_LOW = p.Results.roi_side_th_low; 
    ROI_SIDE_TH_HIGH = p.Results.roi_side_th_high; 
    VERBOSE = p.Results.verbose;
    
    if strcmp(METHOD, 'rdp')
        % Ramer�Douglas�Peucker refinement
        roi_refinement_core = @(roi_raw) roi_refinement_rdp(roi_raw, RDP_TH);
    elseif strcmp(METHOD, 'geometric')
        % Geometric refinement
        roi_refinement_core = @(roi_raw) roi_refinement_geometric(roi_raw);
    else
        % Invalid ROI_REFINEMENT_METHOD
        error('Error: Invalid ROI_REFINEMENT_METHOD = \"%s\"', METHOD);
    end
    
    % Dimension of the diagonal of the image
    diag_img = norm([size(img, 1), size(img, 2)]);
    
    % Select only the valid ROIs among the rois_raw
    rois_refined = cell(0);
    i_rois_refined = [];
    rois_discarded = cell(0);
    for i = 1:size(rois_raw, 1)
        
        roi = rois_raw{i};
        
        % Discard a priori ROIs with only few points
        if size(roi,1) > ROI_SIZE_TH
        
            % Refine ROI with the chosen method
            roi_refined = roi_refinement_core(roi);

            % Check if the ROI is a valid quadrilateral
            if check_quadrilateral(roi_refined, ...
                'sum_angles_tol', ROI_SUM_ANGLES_TOL, ...
                'parallelism_tol', ROI_PARALLELISM_TOL, ...
                'side_th_low', ROI_SIDE_TH_LOW*diag_img, ...
                'side_th_high', ROI_SIDE_TH_HIGH*diag_img) == 1

                % ROI ok, add to rois
                rois_refined{end+1,1} = roi_refined;
                i_rois_refined(end+1) = i;
                
            else
                
                % ROI not valid, discard
                rois_discarded{end+1,1} = roi_refined;

            end
        
        end
        
    end
    
    n_rois_refined = size(rois_refined,1);
    n_rois_discarded = size(rois_discarded,1);
    
    % Plot refined ROIs
    if VERBOSE > 0
        
        figure;
        imshow(img);
        hold on;
        
        colors = winter(n_rois_refined);
        lines_refined_obj = gobjects(1,n_rois_refined);
        lines_refined_str = cell(1,n_rois_refined);
        for k=1:n_rois_refined
           lines_refined_obj(k) = line([rois_refined{k,1}(:,1); rois_refined{k,1}(1,1)], ...
                [rois_refined{k,1}(:,2); rois_refined{k,1}(1,2)], ...
                'color', colors(k,:), ...
                'linestyle', '-', 'linewidth', 1.5, ...
                'marker', 'o', 'markersize', 5);
           lines_refined_str{k} = sprintf('i raw=%3d', i_rois_refined(k));
        end
        
        % Plot discarded ROIs
        if VERBOSE > 1
            lines_discarded_str = 'discarded';
            for k=1:n_rois_discarded
                line_discarded_obj = line([rois_discarded{k,1}(:,1); rois_discarded{k,1}(1,1)], ...
                    [rois_discarded{k,1}(:,2); rois_discarded{k,1}(1,2)], ...
                    'color', 'r', ...
                    'linestyle', '-', 'linewidth', 1.5, ...
                    'marker', 'o', 'markersize', 5);
            end
        end
        
        title(sprintf('Refined ROIs N=%d', n_rois_refined));
        if n_rois_refined > 0
            legend(lines_refined_obj, lines_refined_str{:});
        end
        if n_rois_discarded > 0 && VERBOSE > 1
            legend([lines_refined_obj line_discarded_obj], lines_refined_str{:}, lines_discarded_str);
        end
        
    end

end

function roi_refined = roi_refinement_rdp(roi_raw, RDP_TH)
            
    % Calculate polyfit Ramer�Douglas-Pecker Algorithm
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