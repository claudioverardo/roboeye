function [rois_matched, i_rois_matched, i_arucos] = roi_matching(img, img_gray, rois, aruco_markers, varargin)
% ROI_MATCHING Match Aruco markers with candidate ROIs
%
%   [rois_matched, i_rois_matched, i_arucos] = ROI_MATCHING(img, img_gray, rois, aruco_markers)
%
%   Input arguments:
%   ------------------
%   img:                input image
%   img_gray:           input image grayscale
%   rois:               region of interest candidated for matching with markers
%   aruco_markers:      input marker dictionary
%
%   Parameters:
%   --------
%   'roi_bb_padding':   padding value of bounding boxes
%   'roi_h_side':       side value of ROI after homography
%   'roi_hamming_th':   max value of hamming distance to detect a marker
%   'verbose':           verbose level of the function (allowed values 1, 2, 3)
%
%   Output arguments:
%   ------------------
%   rois_matched:       matched rois among the rois
%   i_rois_matched:     indices of the rois_matched in the rois cell array
%   i_arucos:           indices of the matched marker for every rois matched 
%
%   See also ARUCO_DETECTION
    
    marker_side = size(aruco_markers{1,1},1);
    n_aruco_markers = size(aruco_markers,1);

    % Default values of parameters    
    default_roi_bb_padding = 2;
    default_roi_h_side = 40;
    default_roi_hamming_th = 0;
    default_verbose = 1;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'roi_bb_padding', default_roi_bb_padding, @(x) x>=0);
    addParameter(p, 'roi_h_side', default_roi_h_side, @(x) x>=marker_side);
    addParameter(p, 'roi_hamming_th', default_roi_hamming_th, @(x) x>=0);
    addParameter(p, 'verbose', default_verbose, @(x) x>=0);
    parse(p, varargin{:});
    
    % Parse function parameters
    ROI_BB_PADDING = p.Results.roi_bb_padding; 
    ROI_H_SIDE = p.Results.roi_h_side;
    ROI_HAMMING_TH = p.Results.roi_hamming_th;
    VERBOSE = p.Results.verbose;
    
    % Save the markers rotated by 0°, 90°, 180°, 270°
    aruco_markers_rot = cell(n_aruco_markers, 4);
    for i_aruco=1:n_aruco_markers
        for k_rot=1:4
            aruco_markers_rot{i_aruco, k_rot} = rot90(aruco_markers{i_aruco,1}, k_rot-1);
        end
    end
    
    rois_matched   = cell(0);
    i_rois_matched = [];
    i_arucos = [];
    k_rots   = [];
    
    n_rois = size(rois, 1);
    
    %---------------------------------------------------------------------
    % Change path to avoid conflicts with Fusiello ComputerVisionToolkit
    path = which('imwarp','-all');
    [changedFolder,~,~] = fileparts(path{end});
    currentFolder = pwd;
    cd(changedFolder); % path of Matlab imwarp
    %---------------------------------------------------------------------
    
    % Iterate over all the regions of interest
    for i_roi = 1:n_rois
        
        % Current ROI
        roi = rois{i_roi};

        % Identify the boundary box of the ROI
        bb_idx_top    = max( min(roi(:,2)) - ROI_BB_PADDING, 1 );
        bb_idx_bottom = min( max(roi(:,2)) + ROI_BB_PADDING, size(img,1) );
        bb_idx_left   = max( min(roi(:,1)) - ROI_BB_PADDING, 1 );
        bb_idx_right  = min( max(roi(:,1)) + ROI_BB_PADDING, size(img,2) );
        bb_height     = bb_idx_bottom - bb_idx_top;
        bb_width      = bb_idx_right - bb_idx_left;
        bb_size       = bb_width * bb_height;

        % Binary content of the bounding box
        bb_bw = imbinarize(img_gray(bb_idx_top:bb_idx_bottom,bb_idx_left:bb_idx_right,:));
        R_bb_bw = imref2d(size(bb_bw),[0 bb_width],[0 bb_height]);

        % Set the origin of the ROI vertices to the top-left of the bounding box
        % NOTE: coordinates in the x,y frame, not in the px frame!
        bb_vertices = [ roi(:,1)-bb_idx_left roi(:,2)-bb_idx_top ];

        % Create the ROI control points (a side x side square)
        % NOTE: coordinates in the x,y frame, not in the px frame!
        bb_vertices_H = ROI_H_SIDE * [
            0, 0
            1, 0
            1, 1
            0, 1
        ];
    
        % Compute the homography transformation [Fusiello]
        % H_est = hom_lin(bb_vertices_H', bb_vertices');
        % H_est = hom_nonlin(H_est, bb_vertices_H', bb_vertices');
        % H_est = H_est';
        % H_est_tform = projective2d(H_est);
        
        % Compute the homography transformation [Matlab]
        H_est_tform = fitgeotrans(bb_vertices, bb_vertices_H, 'projective');

        % Transform the content of the bounding box with the homography
        [bb_bw_H, R_bb_bw_H] = imwarp(bb_bw, R_bb_bw, H_est_tform, 'interp', 'nearest');

        % Retrieve the top-left vertex of the transformed ROI in the px frame
        [bb_vertexTL_H_i, bb_vertexTL_H_j] = worldToSubscript(R_bb_bw_H, bb_vertices_H(1,1), bb_vertices_H(1,2));
        
        % Select the content of the ROI within the transformed bounding box
        bb_vertexTL_H_i = max(bb_vertexTL_H_i, 1); % top-left i
        bb_vertexBL_H_i = min(bb_vertexTL_H_i + ROI_H_SIDE - 1, size(bb_bw_H,1)); % bottom-left i
        bb_vertexTL_H_j = max(bb_vertexTL_H_j, 1); % top-left j
        bb_vertexTR_H_j = min(bb_vertexTL_H_j + ROI_H_SIDE - 1, size(bb_bw_H,2)); % top-right j
        
        bb_bw_H_crop = bb_bw_H( ...
            bb_vertexTL_H_i : bb_vertexBL_H_i, ...
            bb_vertexTL_H_j : bb_vertexTR_H_j  ...
        );

        % Downsample to marker_side x marker_side px for aruco matching
        proposed_aruco = imresize(bb_bw_H_crop, [marker_side marker_side]);

        % Search matching with aruco markers
        detected_aruco = 0;
        for i_aruco = 1:n_aruco_markers
            for k_rot=1:4
                aruco_marker = aruco_markers_rot{i_aruco,k_rot};

                % Hamming distance between rotated Aruco and ROI content
                D = pdist( ...
                    cast([proposed_aruco(:) aruco_marker(:)]','double'), ...
                    'hamming' ...
                );

                % Matching
                if D <= ROI_HAMMING_TH / (marker_side*marker_side)
                    detected_aruco = 1;

                    % sort ROI vertices to have the Aruco control point in 1st position
                    % the other points follow the clockwise ordering of the ROI vertices in the image plane
                    roi_sorted = zeros(size(roi));
                    for i_vertex = 1:4
                        roi_sorted(i_vertex,:) = roi(mod(i_vertex-k_rot,4)+1,:);
                    end

                    % save results
                    rois_matched{end+1,1} = roi_sorted;
                    i_rois_matched(end+1) = i_roi;
                    i_arucos(end+1) = i_aruco;
                    k_rots(end+1)   = k_rot;

                    break
                end

            end
            if detected_aruco == 1
                break
            end
        end

        % Plots
        if VERBOSE > 2 || (VERBOSE == 2 && detected_aruco == 1)

            figure;

            % Plot original image with the ROI highlighted
            subplot(2,4,[1,2]);
            imshow(img);
            hold on;
            line([roi(:,1); roi(1,1)], ...
                 [roi(:,2); roi(1,2)], ...
                 'color','r','linestyle','-','linewidth',1.5, ...
                 'marker','o','markersize',5);
            plot(roi(1,1), roi(1,2), 'co', 'MarkerFaceColor', 'c');
            if detected_aruco == 1
                plot(rois_matched{end,1}(1,1), rois_matched{end,1}(1,2), 'go', 'MarkerFaceColor', 'g');
            end
            title('original image');

            % Plot the bw image with the ROI highlighted
            subplot(2,4,[3,4]);
            imshow(img_gray);
            hold on;
            line([roi(:,1); roi(1,1)], ...
                 [roi(:,2); roi(1,2)], ...
                 'color','r','linestyle','-','linewidth',1.5, ...
                 'marker','o','markersize',5);
            title('gray image');

            % Plot the bounding box of the ROI
            subplot(2,4,5);
            imshow(bb_bw, R_bb_bw);
            line([bb_vertices(:,1); bb_vertices(1,1)], ...
                 [bb_vertices(:,2); bb_vertices(1,2)], ...
                 'color','r','linestyle','-','linewidth',1.5, ...
                 'marker','o','markersize',5);
            title('bounding box');

            % Plot the bounding box of the ROI after the homography
            subplot(2,4,6);
            imshow(bb_bw_H, R_bb_bw_H);
            hold on;
            line([bb_vertices_H(:,1); bb_vertices_H(1,1)], ...
                 [bb_vertices_H(:,2); bb_vertices_H(1,2)], ...
                 'color','r','linestyle','-','linewidth',1.5, ...
                 'marker','o','markersize',5);
            title('homography');

            % Plot the content of the vertices downsampled to marker_side x marker_side px
            subplot(2,4,7);
            imshow(proposed_aruco);
            hold on;
            plot(1, 1, 'co', 'MarkerFaceColor', 'c');
            title('proposal');

            % Plot the content of the vertices downsampled to marker_side x marker_side px
            subplot(2,4,8);
            if detected_aruco == 1
                imshow(aruco_markers_rot{i_aruco,k_rot});
                hold on;
                coltrol_point_aruco = [
                              1           1
                              1 marker_side
                    marker_side marker_side
                    marker_side           1
                ];
                plot(coltrol_point_aruco(k_rot,1), coltrol_point_aruco(k_rot,2), 'go', 'MarkerFaceColor', 'g');
                title(sprintf('detected i=%d %d°',i_aruco,(k_rot-1)*90));
            else
                imshow(zeros(marker_side, marker_side));
                title(sprintf('detected NaN'));
            end

            suptitle(sprintf('ROI Matching %d / %d', i_roi, n_rois));
            annotation( 'textbox', ...
                'string', 'red: roi      cyan: detected control point      green: aruco control point', ...
                'Position', [0, 0.5, 1, 0], ...
                'HorizontalAlignment', 'center', ...
                'LineStyle', 'none' ...
            );

        end

    end

    %--------------------------------------------------------------
    % Back to original path
    cd(currentFolder);
    %--------------------------------------------------------------
    
    n_rois_matched = size(rois_matched,1);

    % Plot matched ROIs
    if VERBOSE > 0
        figure;
        imshow(img);
        
        colors = hsv(n_rois_matched);
        lines_obj = gobjects(1,n_rois_matched);
        lines_str = cell(1,n_rois_matched);
        for k=1:n_rois_matched
           hold on;
           lines_obj(k) = line( ...
                [rois_matched{k,1}(:,1); rois_matched{k,1}(1,1)], ...
                [rois_matched{k,1}(:,2); rois_matched{k,1}(1,2)], ...
                'color', colors(k,:), ...
                'linestyle', '-', 'linewidth', 1.5, ...
                'marker', 'o', 'markersize', 5 ...
           );
           lines_str{k} = sprintf('ROI=%d, Aruco=%d', k, i_arucos(k));
           point_obj = plot(rois_matched{k,1}(1,1), rois_matched{k,1}(1,2), 'gs', 'MarkerSize', 10, 'LineWidth', 1); % 'MarkerFaceColor', 'g'
        end
        title(sprintf('Matched ROIs N=%d', size(rois_matched,1)));
        if n_rois_matched > 0
            legend([lines_obj point_obj], lines_str{:}, 'Control points');
        end
    end
    
end