function cumCorrScore = tempMatching(frameImg, templateImg)
    % -------------------------------------------------------------------------
    % Function corrMatching: Template Matching using Correlation Coefficients
    % Inputs:
    %           frameImg = gray or color frame image
    %           templateImg = gray or color template image
    %           threshC = threshold of rejecting detected region (default = .75)
    %                     e.g. if the detected region has a corrCoef>threshC
    %                     then the algorithm accepts it as a detection,
    %                     otherwise rejects it as a false alarm.
    % Output:
    %           corrScore = 2D matrix of correlation coefficients
    %
    % -------------------------------------------------------------------------
    % By Yue Wu (Rex)
    % Department of Electrical and Computer Engineering
    % Tufts University
    % Medford, MA
    % 08/30/2010
    % -------------------------------------------------------------------------
    % Mofified by A. Fusiello, 2019
    
    %% 1. initialization
    if size(frameImg,3) ~=1
        frameGray = rgb2gray(frameImg);
    else
        frameGray = frameImg;
    end
    frameGray = double(frameGray);
    
    if size(templateImg,3) ~=1
        templateGray = rgb2gray(templateImg);
    else
        templateGray = templateImg;
    end
    templateGray = double(templateGray);
    
    
    %% 2. correlation calculation (rotate the same template 4 times by 90 deg)
    frameMean = conv2(frameGray,ones(size(templateGray))./numel(templateGray),'same');
    templateMean = mean(templateGray(:));
    stdTemplate = std(templateGray(:));
    stdFrame = sqrt(conv2(frameGray.^2,ones(size(templateGray))./numel(templateGray),'same')-frameMean.^2);
    corrPartII = frameMean.*sum(templateGray(:)-templateMean);
    
    corrPartI = conv2(frameGray,rot90(templateGray-templateMean,1),'same')./numel(templateGray);
    corrScore =  (corrPartI-corrPartII)./(stdFrame.*stdTemplate);
   
    cumCorrScore = corrScore ; 
    
    corrPartI = conv2(frameGray,rot90(templateGray-templateMean,2),'same')./numel(templateGray);
    corrScore =  (corrPartI-corrPartII)./(stdFrame.*stdTemplate);
    
    cumCorrScore =  max(cumCorrScore,corrScore); 
    
    corrPartI = conv2(frameGray,rot90(templateGray-templateMean,3),'same')./numel(templateGray);
    corrScore =  (corrPartI-corrPartII)./(stdFrame.*stdTemplate);
    
    cumCorrScore =  max(cumCorrScore,corrScore); 
    
    corrPartI = conv2(frameGray,templateGray-templateMean,'same')./numel(templateGray);
    corrScore =  (corrPartI-corrPartII)./(stdFrame.*stdTemplate);

    cumCorrScore =  max(cumCorrScore,corrScore); 
    
    
