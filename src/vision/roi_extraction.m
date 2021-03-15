function rois_raw = roi_extraction(img, img_bw, img_gray, varargin)

    % Default values of parameters
    default_method = 'adaptth-moore';
    default_canny_th_low = 0.01;
    default_canny_th_high = 0.10;
    default_verbose = 0;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'method', default_method);
    addParameter(p, 'canny_th_low', default_canny_th_low);
    addParameter(p, 'canny_th_high', default_canny_th_high);
    addParameter(p, 'verbose', default_verbose);
    parse(p, varargin{:});
    
    % Parse function parameters
    METHOD = p.Results.method;
    CANNY_TH_LOW = p.Results.canny_th_low;
    CANNY_TH_HIGH = p.Results.canny_th_high;
    VERBOSE = p.Results.verbose;

    if strcmp(METHOD, 'adaptth-moore')
        
        % Binarization already performed in aruco_detection
        % img_th = adaptthresh(img_gray, 1, 'Statistic', 'gaussian');
        % img_bw = imbinarize(img_gray, img_th);
    
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
                'color','r','linestyle','-','linewidth',1.5, ...
                'marker','o','markersize',5);
        end
        title(sprintf('Extracted ROIs N=%d', size(rois_raw,1)));
    end

end