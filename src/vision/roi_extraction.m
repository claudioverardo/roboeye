function rois_raw = roi_extraction(img, img_gray, varargin)
% ROI_EXTRACTION Extract ROIs from input image
%
%   rois_raw = ROI_EXTRACTION(img, img_gray)
%
%   Input arguments:
%   ------------------
%   img:                    input image
%   img_gray:               input image grayscale
%
%   Parameters:
%   ------------------   
%   'method':				choose the roi extraction algorithm
%                           'adaptth-moore': Adaptive thresholding + Moore-Neighbor tracing 
%                           'canny-dfs': Canny edge detector + DFS
%                           'canny-dfs-c': Canny edge detector + DFS C-implementation   
%   'adaptth_sensitivity':	sensitivity of the adaptive thresholding (cf adaptthresh)	
%   'adaptth_statistic':	statistic of the adaptive thresholding (cf adaptthresh)			
%   'adaptth_neighborhood':	neighborhood of the adaptive thresholding (cf adaptthresh)			
%   'canny_th_low':			lower threshold of the Canny edge detector (cf edge)
%   'canny_th_high':		higher threshold of the Canny edge detector (cf edge)		
%   'verbose:               verbose level of the function (allowed values 0, 1, 2)
%
%   Output arguments:
%   ------------------
%   rois_raw:               extracted rois without any refinement
%
%   See also ARUCO_DETECTION

    % Default values of parameters
    default_method = 'adaptth-moore';
    default_adaptth_sensitivity = 1;
    default_adaptth_statistic = 'gaussian';
    default_adaptth_neighborhood = 2*floor(size(img_gray)/16)+1;
    default_canny_th_low = 0.01;
    default_canny_th_high = 0.10;
    default_verbose = 0;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'method', default_method);
    addParameter(p, 'adaptth_sensitivity', default_adaptth_sensitivity);
    addParameter(p, 'adaptth_statistic', default_adaptth_statistic);
    addParameter(p, 'adaptth_neighborhood', default_adaptth_neighborhood);
    addParameter(p, 'canny_th_low', default_canny_th_low);
    addParameter(p, 'canny_th_high', default_canny_th_high);
    addParameter(p, 'verbose', default_verbose);
    parse(p, varargin{:});
    
    % Parse function parameters
    METHOD = p.Results.method;
    ADAPTTH_SENSITIVITY = p.Results.adaptth_sensitivity;
    ADAPTTH_STATISTIC = p.Results.adaptth_statistic;
    ADAPTTH_NEIGHBORHOOD = p.Results.adaptth_neighborhood;
    CANNY_TH_LOW = p.Results.canny_th_low;
    CANNY_TH_HIGH = p.Results.canny_th_high;
    VERBOSE = p.Results.verbose;

    if strcmp(METHOD, 'adaptth-moore')
    
        % Conveter image to binary
        % sensitivity -> [0,1]
        % Statistic -> mean, gaussian, median
        % NeighborhoodSize -> [height, width,], odd values, default 2*floor(size(img_gray)/16)+1
        img_th = adaptthresh(img_gray, ADAPTTH_SENSITIVITY, 'Statistic', ADAPTTH_STATISTIC, 'NeighborhoodSize', ADAPTTH_NEIGHBORHOOD); 
        img_bw = imbinarize(img_gray, img_th);
    
        % Plot binarization output
        if VERBOSE > 1
            figure;
            imshow(img_bw);
            title('Binarized image');
        end
        
        % Extract morphological components - Moore-Neighbor tracing
        rois_raw = bwboundaries(1-img_bw,'noholes');
        
        for k=1:size(rois_raw,1)
            % ij --> xy coordinates
            rois_raw{k} = circshift(rois_raw{k},1,2);
            % remove the last point (equal to the first one)
            rois_raw{k}(end,:) = [];
            % NOTE: the output of bwboundaries may contain repeated points (pay attention with roi_refinement_geometric)
        end
        
    elseif strcmp(METHOD, 'canny-dfs')
        
        % Grayscale converion already performed in aruco_detection
        % img_gray = rgb2gray(img);

        % Canny edge detector
        img_canny = edge(img_gray, 'canny', [CANNY_TH_LOW, CANNY_TH_HIGH]);

        % Plot Canny output
        if VERBOSE > 1
           figure;
           imshow(img_canny);
           title('Canny Edge Detector');
        end

        % Extract morphological components - DFS [MATLAB-implementation]
        components = roi_extraction_dfs(img_canny);
        
        if VERBOSE > 0
            I = zeros(size(img));
            for i = 1:size(components, 1)
                % plot component points
                for j = 1:size(components{i, 1}, 1)
                    I(components{i, 1}(j, 2), components{i, 1}(j, 1), :) = [255, 255, 255];
                end
                % plot tails
                for j = 1:size(components{i, 2}, 1)
                    I(components{i, 2}(j, 2), components{i, 2}(j, 1), :) = [255, 0, 0];
                end
            end

            figure;
            imshow(I);
            title('Components');
        end
        
        rois_raw = components(:,1);
        
    elseif strcmp(METHOD, 'canny-dfs-c')
        
        % Grayscale converion already performed in aruco_detection
        % img_gray = rgb2gray(img);

        % Canny edge detector
        img_canny = edge(img_gray, 'canny', [CANNY_TH_LOW, CANNY_TH_HIGH]);

        % Plot Canny output
        if VERBOSE > 1
           figure;
           imshow(img_canny);
           title('Canny Edge Detector');
        end

        % Extract morphological components - DFS [C-implementation]
        [components, tails] = roi_extraction_dfs_c(img_canny, size(img_canny, 1), size(img_canny, 2));
        rois_raw = components;
    
    else
        
        % Invalid ROI_EXTRACTION_METHOD
        error('Error: Invalid ROI_EXTRACTION_METHOD = \"%s\"', METHOD);
    
    end
    
    % Plot extracted ROIs
    if VERBOSE > 0
        figure;
        imshow(img);
        for k=1:size(rois_raw,1)
           hold on;
           line([rois_raw{k,1}(:,1); rois_raw{k,1}(1,1)], ...
                [rois_raw{k,1}(:,2); rois_raw{k,1}(1,2)], ...
                'color',[rand rand rand],'linestyle','-','linewidth',1.5, ...
                'marker','o','markersize',5);
        end
        title(sprintf('Extracted ROIs N=%d', size(rois_raw,1)));
    end

end