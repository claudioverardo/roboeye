function [rois_matched, i_rois, i_arucos, k_rots] = roi_matching(img, rois, aruco_markers, varargin)
% ARUCO_MATCHING  Matching of aruco markers in an image.
%   ARUCO_MATCHING(IMG, ROIS, ARUCO_MARKERS) match the ARUCO_MARKERS with the ROIS of IMG 
%
%   Parameters:
%   --------
%
%   'bb_padding'        padding value of bounding boxes
%
%   'roi_th_size'       threshold on ROI size (fraction of the image size)
%
%   'roi_h_side'        side value of ROI after homography
%
%   'hamming_th'        max value of hamming distance to detect a marker
%
%   'verbose'           launch function in verbose mode
%
%   Examples
%   --------
%   TODO TODO
%
%   See also GET_MORPHOLOGICAL_COMPONENTS

    img_size = size(img,1)*size(img,2);
    img_bw = imbinarize(rgb2gray(img));
    marker_side = size(aruco_markers,1);
    n_aruco_markers = size(aruco_markers,3);

    % Default values of parameters    
    default_bb_padding  = 2;
    default_roi_th_size = 0.5;
    default_roi_h_side  = 40;
    default_hamming_th  = 0;
    default_verbose     = 1;
    
    % Input parser
    p = inputParser;
    addParameter(p, 'bb_padding', default_bb_padding, @(x) x>=0);
    addParameter(p, 'roi_th_size', default_roi_th_size, @(x) x>=0);
    addParameter(p, 'roi_h_side', default_roi_h_side, @(x) x>=marker_side);
    addParameter(p, 'hamming_th', default_hamming_th, @(x) x>=0);
    addParameter(p, 'verbose', default_verbose, @(x) x>=0);
    parse(p, varargin{:});
    
    % Parse function parameters
    BB_PADDING  = p.Results.bb_padding; 
    ROI_TH_SIZE = p.Results.roi_th_size; 
    ROI_H_SIDE  = p.Results.roi_h_side;
    HAMMING_TH  = p.Results.hamming_th;
    VERBOSE     = p.Results.verbose;
    
    % Save the markers rotated by 0°, 90°, 180°, 270°
    aruco_markers_rot = zeros(marker_side, marker_side, 4, n_aruco_markers);
    for i_aruco=1:n_aruco_markers
        for k_rot=1:4
            aruco_markers_rot(:,:,k_rot,i_aruco) = rot90(aruco_markers(:,:,i_aruco),k_rot-1);
        end
    end
    
    rois_matched = zeros(4,2,0);
    i_rois   = [];
    i_arucos = [];
    k_rots   = [];
    
    % Iterate over all the regions of interest
    for i_roi = 1:size(rois, 1)
        
        % Current ROI
        roi = rois{i_roi};

        % Check if the ROI is a convex quadrilateral
        if check_cvx_quadrilateral(roi) == 1

            % Identify the boundary box of the ROI
            bb_idx_top    = max( min(roi(:,2)) - BB_PADDING, 1 );
            bb_idx_bottom = min( max(roi(:,2)) + BB_PADDING, size(img,1) );
            bb_idx_left   = max( min(roi(:,1)) - BB_PADDING, 1 );
            bb_idx_right  = min( max(roi(:,1)) + BB_PADDING, size(img,2) );
            bb_height     = bb_idx_bottom - bb_idx_top;
            bb_width      = bb_idx_right - bb_idx_left;
            bb_size       = bb_width * bb_width;

            % Check if the boundary box has very huge dimensions
            if bb_size < ROI_TH_SIZE * img_size

                % Binary content of the bounding box
                bb_bw = img_bw(bb_idx_top:bb_idx_bottom,bb_idx_left:bb_idx_right,:);
                R_bb_bw = imref2d(size(bb_bw),[0 bb_width],[0 bb_height]);

                % Set the origin of the ROI vertices to the top-left of the bounding box
                % NOTE: coordinates in the x,y frame, not in the px frame!
                bb_vertices = [ roi(:,1)-bb_idx_left roi(:,2)-bb_idx_top ];
    
                % ensure that bb_vertices are sorted in a clockwise order, from top-left in the bounding box
                % example of problematic case:
                % bb_vertices = [ [20 5]; [16 27]; [5 32]; [14 6] ]
                % bb_vertices_norm = vecnorm(bb_vertices');
                % [~, bb_vertices_norm_argmin] = min(bb_vertices_norm);
                % bb_vertices = [ 
                %     bb_vertices(bb_vertices_norm_argmin:end,:)
                %     bb_vertices(1:bb_vertices_norm_argmin-1,:) 
                % ];

                % Compute homography to the side x side square
                % NOTE: coordinates in the x,y frame, not in the px frame!
                bb_vertices_H = ROI_H_SIDE * [
                    0, 0
                    1, 0
                    1, 1
                    0, 1
                ];
                H_est = hom_lin(bb_vertices_H', bb_vertices');
                H_est = hom_nonlin(H_est, bb_vertices_H', bb_vertices');

                %--------------------------------------------------------------
                % Change path to use Matlab imwarp
                path = which('imwarp','-all');
                [changedFolder,~,~] = fileparts(path{end});
                currentFolder = pwd;
                cd(changedFolder);
                %--------------------------------------------------------------

                % Transform the content of the bounding box with the homography
                tform = projective2d(H_est');
                [bb_bw_H, R_bb_bw_H] = imwarp(bb_bw, R_bb_bw, tform);

                %--------------------------------------------------------------
                % Back to original path for imwarp
                cd(currentFolder);
                %--------------------------------------------------------------
                
                % Retrieve the top-left vertex of the transformed ROI in the px frame
                [bb_vertexTL_H_i, bb_vertexTL_H_j] = worldToSubscript(R_bb_bw_H, bb_vertices_H(1,1), bb_vertices_H(1,2));
                
                % Select the content of the ROI within the transformed bounding box
                bb_bw_H_crop = bb_bw_H( ...
                    bb_vertexTL_H_i : bb_vertexTL_H_i + ROI_H_SIDE - 1, ...
                    bb_vertexTL_H_j : bb_vertexTL_H_j + ROI_H_SIDE - 1  ...
                );
            
                % Downsample to marker_side x marker_side px for aruco matching
                proposed_aruco = imresize(bb_bw_H_crop, [marker_side marker_side]);
                
                % Search matching with aruco markers
                detected_aruco = 0;
                for i_aruco = 1:n_aruco_markers
                    for k_rot=1:4
                        aruco_marker = aruco_markers_rot(:,:,k_rot,i_aruco);
                        
                        % Hamming distance between rotated Aruco and ROI content
                        D = pdist( ...
                            cast([proposed_aruco(:) aruco_marker(:)]','double'), ...
                            'hamming' ...
                        );
                    
                        % Matching ?
                        if D <= HAMMING_TH / (marker_side*marker_side)
                            detected_aruco = 1;
                            
                            % sort ROI vertices to have the Aruco control point in 1st position
                            % the other points follow the clockwise ordering of the ROI vertices in the image plane
                            roi_sorted = zeros(size(roi));
                            for i_vertex = 1:4
                                roi_sorted(i_vertex,:) = roi(mod(i_vertex-k_rot,4)+1,:);
                            end
                            
                            % save results
                            rois_matched(:,:,end+1) = roi_sorted;
                            i_rois   = [i_rois, i_roi];
                            i_arucos = [i_arucos, i_aruco];
                            k_rots   = [k_rots, k_rot];
                            
                            break
                        end
                        
                    end
                    if detected_aruco == 1
                        break
                    end
                end

                % Plots
                if VERBOSE > 1 || (VERBOSE== 1 && detected_aruco ==1)
                    
                    % Plot original image with the ROI highlighted
                    figure;
                    subplot(2,4,[1,2]);
                    imshow(img);
                    hold on;
                    line([roi(:,1); roi(1,1)], ...
                         [roi(:,2); roi(1,2)], ...
                         'color','r','linestyle','-','linewidth',1.5, ...
                         'marker','o','markersize',5);
                    plot(roi(1,1), roi(1,2), 'co', 'MarkerFaceColor', 'c');
                    if detected_aruco == 1
                        plot(rois_matched(1,1,end), rois_matched(1,2,end), 'go', 'MarkerFaceColor', 'g')
                    end
                    title(sprintf('original i=%d', i_roi));

                    % Plot the bw image with the ROI highlighted
                    subplot(2,4,[3,4]);
                    imshow(img_bw);
                    hold on;
                    line([roi(:,1); roi(1,1)], ...
                         [roi(:,2); roi(1,2)], ...
                         'color','r','linestyle','-','linewidth',1.5, ...
                         'marker','o','markersize',5);
                    title('binary');
                    
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
                    title('after H');
                    % further check: retrieve the bb_vertices_H as H_est * bb_vertices and plot
                    % bb_vertices_H_check = htx(H_est, bb_vertices')';
                    % line([bb_vertices_H_check(:,1); bb_vertices_H_check(1,1)], ...
                    %      [bb_vertices_H_check(:,2); bb_vertices_H_check(1,2)], ...
                    %      'color','g','linestyle','-','linewidth',1.5, ...
                    %      'marker','o','markersize',5);

                    % Plot only the ROI after the homography
                    % subplot(2,4,7);
                    % imshow(bb_bw_H_crop);
                    % title('crop');

                    % Plot the content of the vertices downsampled to marker_side x marker_side px
                    subplot(2,4,7);
                    imshow(proposed_aruco);
                    hold on;
                    plot(1, 1, 'co', 'MarkerFaceColor', 'c')
                    title('proposal');

                    % Plot the content of the vertices downsampled to marker_side x marker_side px
                    subplot(2,4,8);
                    if detected_aruco == 1
                        imshow(aruco_markers_rot(:,:,k_rot,i_aruco));
                        hold on;
                        coltrol_point_aruco = [
                                      1           1
                                      1 marker_side
                            marker_side marker_side
                            marker_side           1
                        ];
                        plot(coltrol_point_aruco(k_rot,1), coltrol_point_aruco(k_rot,2), 'go', 'MarkerFaceColor', 'g')
                        title(sprintf('detected %d %d°',i_aruco,(k_rot-1)*90));
                    else
                        imshow(zeros(marker_side, marker_side));
                        title(sprintf('detected NaN'));
                    end
                
                end

            end
            
        end

    end
    
end