function rois = roi_refinement(rois_raw, RDP_TH)

    rois = cell(0);

    for i = 1:size(rois_raw, 1)
        
        % Calculate polyfit Douglas Pecker Algorithm
        roi = reducepoly(rois_raw{i}, RDP_TH);  
        
        % Remove last point (= to the first one)
        roi(end,:) = [];
        
        if check_cvx_quadrilateral(roi) == 1
        
            rois{size(rois,1)+1,1} = roi;

        end
        
    end

end