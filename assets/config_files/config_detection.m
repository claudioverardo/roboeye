%% CONFIGURATION FILE OF ROBOEYE (DETECTION)

% ROI extraction parameters
ROI_EXTRACTION_METHOD = 'adaptth-moore'; % adaptth-moore, canny-dfs, canny-dfs-c
ADAPTTH_SENSITIVITY = 0.7;        % [0,1]
ADAPTTH_STATISTIC = 'gaussian';   % mean, gaussian, median
ADAPTTH_NEIGHBORHOOD = [135 241]; % default for 1080x1920 -> [135 241]
CANNY_TH_LOW  = 0.01;
CANNY_TH_HIGH = 0.10;

% ROI refinement parameters
ROI_REFINEMENT_METHOD = 'geometric'; % rdp, geometric
ROI_SIZE_TH = 50;
RDP_TH = 0.2;  % Ramer–Douglas–Peucker threshold
ROI_SUM_ANGLES_TOL  = 10; % [degrees]
ROI_PARALLELISM_TOL = 10; % [degrees]
ROI_SIDE_TH_LOW  = 1/100; % [% diag(img)]
ROI_SIDE_TH_HIGH = 1/5;   % [% diag(img)]
ROI_ANGLE_TH_LOW = 20;    % [degrees]
ROI_ANGLE_TH_HIGH = 160;  % [degrees]

% ROI matching parameters
ROI_BB_PADDING  = 2;
ROI_H_SIDE = 80;
ROI_HAMMING_TH  = 2;

% Debug/Analysis

% 1: show roi_extracted
% 2: + show adaptth or canny/dfs 
ROI_EXTRACTION_VERBOSE = 1;

% 1: show roi_refined
% 2: + show roi_discarded
ROI_REFINEMENT_VERBOSE = 1;

% 1: show roi_matched/markers
% 2: + show H (roi_matched)
% 3: + show H (all)
ROI_MATCHING_VERBOSE = 1;

% 1: detection log
% 2: advanced detection log
ARUCO_DETECTION_VERBOSE = 2;
