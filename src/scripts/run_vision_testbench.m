close all; % clear
diary '../assets/vision_testbench/vision_testbench_log.txt';

%% POSE ESTIMATION EVALUATION
%--------------------------------------------------------------------------

fprintf('\n-------- Vision testbench start --------\n');

% cam = webcam(1);

% Directories
aruco_images_dir = '../assets/vision_testbench/aruco_images';
aruco_images_acquired_dir = '../assets/vision_testbench/aruco_images_acquired';
results_pipeline_dir = { '../assets/vision_testbench/results_adaptth_moore' 
                         '../assets/vision_testbench/results_canny_dfs' };
pipeline = {'adaptth-moore', 'canny-dfs'};
n_pipeline = length(pipeline);

if ~exist(aruco_images_dir, 'dir')
    error('ERROR: aruco_images_dir not found!!');
end

% Create folders
if ~exist(aruco_images_acquired_dir, 'dir')
   mkdir(aruco_images_acquired_dir);
end

for j=1:n_pipeline
    if ~exist(results_pipeline_dir{j}, 'dir')
       mkdir(results_pipeline_dir{j});
    end
end

% Testbench data
[files, i_aruco_gt, aruco_real_sides, theta_gt, centroid_gt] = get_testbench_data(aruco_images_dir);
n_files = length(files);
i_aruco_gt = id2id_dict(i_aruco_gt);
aruco_real_sides = aruco_real_sides / 10;

% Vision data
aruco_markers = load('../assets/aruco_markers/aruco_markers_vision_testbench.mat').aruco_markers;
K = load('../assets/calibration/intrinsics_cam2/K.mat').K';
R_cam = load('../assets/calibration/extrinsics_cam2_vision_testbench/R_cam.mat').R_cam';
t_cam = load('../assets/calibration/extrinsics_cam2_vision_testbench/t_cam.mat').t_cam';
k = load('../assets/calibration/intrinsics_cam2/intrinsics.mat').intrinsics.radial;

% Results data structs
check_id = zeros(n_files,n_pipeline);
reproj_err_lin = zeros(n_files,n_pipeline);
reproj_err_nonlin = zeros(n_files,n_pipeline);
centroid = zeros(n_files,3,n_pipeline);
centroid_err = zeros(n_files,3,n_pipeline);
centroid_err_norm = zeros(n_files,n_pipeline);
theta_gt = theta_gt';
theta = zeros(n_files,n_pipeline);
theta_err = zeros(n_files,n_pipeline);
times = zeros(n_files,4,n_pipeline);
times_tot = zeros(n_files,n_pipeline);
n_rois_extracted = zeros(n_files,n_pipeline);
n_rois_refined = zeros(n_files,n_pipeline);

% Launch tests
for i = 1:n_files
% for i = 1:1
    
    fprintf('\nImage %d/%d: %s\n', i, n_files, files(i).name);    
    image_i = fullfile(aruco_images_dir,files(i).name);
    image_acquired_i = fullfile(aruco_images_acquired_dir,files(i).name);
    
    new_acquisition = ~isfile(image_acquired_i);
        
    if new_acquisition
        fprintf('Loading %s\n', image_i);
        img = imread(image_i);
        fig = figure('WindowState', 'fullscreen', 'MenuBar', 'none', 'ToolBar', 'none');
        ax = axes('Units','Normalize','Position',[0 0 1 1]);
        imshow(img);
        fprintf('Waiting... ');
        print_countdown(3);
        fprintf('Acquiring %s\n', image_acquired_i);
        img_cam = snapshot(cam);
        imwrite(img_cam, image_acquired_i);
    else
        fprintf('Loading %s\n', image_acquired_i);
        img_cam = imread(image_acquired_i);
    end
    
    for j=1:n_pipeline
        
        fprintf('Test pipeline: %s\n', pipeline{j});
        image_result_i = fullfile(results_pipeline_dir{j},files(i).name);
    
        [~, i_arucos, rois_R, rois_t, stats] = aruco_pose_estimation( ...
            img_cam, aruco_markers, aruco_real_sides, K, R_cam, t_cam, k, ...
            'roi_extraction_method', pipeline{j}, ...
            'adaptth_sensitivity', 0.7, ...
            'adaptth_statistic', 'gaussian', ...
            'adaptth_neighborhood', [135 241], ...
            'canny_th_low', 0.01, ...
            'canny_th_high', 0.10, ...
            'roi_refinement_method', 'geometric', ...
            'roi_size_th', 50, ...
            'rdp_th', 0.2, ...
            'roi_sum_angles_tol', 10, ...
            'roi_parallelism_tol', 10, ...
            'roi_side_th_low', 1/100, ...
            'roi_side_th_high', 1/5, ...
            'roi_angle_th_low', 20, ...
            'roi_angle_th_high', 160, ...
            'roi_bb_padding', 2, ...
            'roi_h_side', 80, ...
            'roi_hamming_th', 2, ...
            'roi_extraction_verbose', 0, ...
            'roi_refinement_verbose', 0, ...
            'roi_matching_verbose', 0, ...
            'roi_pose_estimation_verbose', 2, ...
            'aruco_detection_verbose', 0, ...
            'verbose', 0 ...
        );
        
        if numel(rois_t) == 0
            fprintf('    ERROR: marker not found!!\n');
        else
            
            if numel(rois_t) > 1
                fprintf('    WARNING: found more than 1 marker!!\n');
            end
            
            % Only one marker in the scene is assumed
            t = rois_t{1};
            R = rois_R{1};
            i_aruco = i_arucos(1);
            roi_reproj_err_lin = stats.reproj_err_lin(1);
            roi_reproj_err_nonlin = stats.reproj_err_nonlin(1);
            fprintf('    Found marker id %d\n', i_aruco);

            % Save the plot of aruco_pose_estimation()
            fprintf('    Saving results image\n');
            saveas(gcf, image_result_i);

            % Align z-axes of marker and checkerboard
            z_axis_marker = [0 0 1]*R;
            x_axis_marker = [1 0 0]*R;
            e = cross([0 0 1], z_axis_marker);
            gamma = acos(dot([0 0 1], z_axis_marker));
            R_gamma = axang2rotm([e gamma]);
            z_axis_marker_fix = z_axis_marker*R_gamma; % check
            x_axis_marker_fix = x_axis_marker*R_gamma;

            % Evaluation parameters
            check_id(i,j) = i_aruco == i_aruco_gt(i);
            reproj_err_lin(i,j) = roi_reproj_err_lin;
            reproj_err_nonlin(i,j) = roi_reproj_err_nonlin;
            centroid(i,:,j) = t;
            centroid_err(i,:,j) = centroid(i,:,j)-centroid_gt(i,:);
            centroid_err_norm(i,j) = norm(centroid_err(i,:,j));
            theta(i,j) = -atan2d(x_axis_marker_fix(2),x_axis_marker_fix(1));
            if theta(i,j) < 0 % -atan2d assume values in -pi < ... <= pi
                theta(i,j) = theta(i,j) + 360;
            end
            theta_err(i,j) = theta(i,j)-theta_gt(i); % NOTE: theta_gt>0 in clockwise direction
            times(i,:,j) = [stats.time_roi_extraction, stats.time_roi_refinement, stats.time_roi_matching, stats.time_roi_pose_estimation];
            times_tot(i,j) = sum(times(i,:,j));
            n_rois_extracted(i,j) = stats.n_rois_extracted;
            n_rois_refined(i,j) = stats.n_rois_refined;

            fprintf('    Reproj error ___lin [px] : %g\n', reproj_err_lin(i,j));
            fprintf('    Reproj error nonlin [px] : %g\n', reproj_err_nonlin(i,j));
            fprintf('    Centroid error  xyz [cm] : %s\n', mat2str(centroid_err(i,:,j),4));
            fprintf('    Centroid error norm [cm] : %g\n', centroid_err_norm(i,j));
            fprintf('    Aligned theta error  [°] : %g\n', theta_err(i,j));
            fprintf('    Execution times      [s] : %s\n', mat2str(times(i,:,j),4));
            fprintf('    Total execution time [s] : %g\n', times_tot(i,j));
            fprintf('    # ROIs extracted     [ ] : %d\n', n_rois_extracted(i,j));
            fprintf('    # ROIs refined       [ ] : %d\n', n_rois_refined(i,j));
            
        end
    
    end
    
    close all;
    
    if new_acquisition
        fprintf('Waiting... ');
        print_countdown(3);
    end

end

columns_results = { ...
    'image', ...
    'check_id', ...
    'reproj_err_lin', ...
    'reproj_err_nonlin', ...
    'centroid_gt', ...
    'centroid', ...
    'centroid_err', ...
    'centroid_err_norm', ...
    'theta_gt', ...
    'theta', ...
    'theta_err' ...
    'times', ...
    'times_tot', ...
    'n_rois_extracted', ...
    'n_rois_refined' ...
};

columns_results_avg = { ...
    'reproj_err_lin', ...
    'reproj_err_nonlin', ...
    'centroid_err', ...
    'centroid_err_norm', ...
    'theta_err', ...
    'times', ...
    'times_tot', ...
    'n_rois_extracted', ...
    'n_rois_refined' ...
};

for j=1:n_pipeline
    
    % Put evaluation parameters in a table
    results = table( ...
        {files.name}', check_id(:,j), ...
        reproj_err_lin(:,j), reproj_err_nonlin(:,j), ...
        centroid_gt, centroid(:,:,j), centroid_err(:,:,j), centroid_err_norm(:,j), ...
        theta_gt, theta(:,j), theta_err(:,j), ...
        times(:,:,j), times_tot(:,j), ...
        n_rois_extracted(:,j), n_rois_refined(:,j), ...
        'VariableNames', columns_results ...
    );

    % RMS values of results
    results_rms = {
        rms(reproj_err_lin(:,j)), rms(reproj_err_nonlin(:,j)), ...
        rms(centroid_err(:,:,j),1), rms(centroid_err_norm(:,j)), ...
        rms(theta_err(:,j)), ...
        rms(times(:,:,j),1), rms(times_tot(:,j)), ...
        rms(n_rois_extracted(:,j)), rms(n_rois_refined(:,j)) ...
    };

    % MEAN values of results
    results_mean = {
        mean(reproj_err_lin(:,j)), mean(reproj_err_nonlin(:,j)), ...
        mean(centroid_err(:,:,j),1), mean(centroid_err_norm(:,j)), ...
        mean(theta_err(:,j)), ...
        mean(times(:,:,j),1), mean(times_tot(:,j)), ...
        mean(n_rois_extracted(:,j)), mean(n_rois_refined(:,j)) ...
    };

    % STD values of results
    results_std = {
        std(reproj_err_lin(:,j)), std(reproj_err_nonlin(:,j)), ...
        std(centroid_err(:,:,j),[],1), std(centroid_err_norm(:,j)), ...
        std(theta_err(:,j)), ...
        std(times(:,:,j),[],1), std(times_tot(:,j)), ...
        std(n_rois_extracted(:,j)), std(n_rois_refined(:,j)) ...
    };

    % Put RMS, MEAN, STD values in a table
	results_avg = cell2table( ...
        {results_rms{:}; results_mean{:}; results_std{:}}, ...
        'VariableNames', columns_results_avg, ...
        'RowNames', {'rms','mean','std'} ...
    );

    fprintf('\nSaving results of %s...\n\n', pipeline{j});
    save(fullfile(results_pipeline_dir{j},'results'), 'results');
    save(fullfile(results_pipeline_dir{j},'results_avg'), 'results_avg');
    
    disp(results);
    disp(results_avg);

end

diary off;

function [files, id, real_side, theta, points] = get_testbench_data(dir_path)

    img_formats = cat(2,imformats().ext);

    files = dir(dir_path);
    files = files(~[files.isdir]);
    filenames = {files.name};

    exts = [];
    filenames_noext = [];
    id = [];
    real_side = [];
    theta = [];
    x = [];
    y = [];
    i = 1;
    while i<=length(files)
        [~,filenames_noext{i},exts{i}] = fileparts(strcat('./',filenames{i}));
        exts{i} = lower(exts{i}(2:end));
        if ismember(exts{i},img_formats)
            parsed_name = split(filenames_noext{i},'_');
            if length(parsed_name) == 6
                id(i) = int32(str2double(parsed_name(2)));
                real_side(i) = str2double(parsed_name(3));
                theta(i) = str2double(parsed_name(4));
                x(i) = str2double(parsed_name(5));
                y(i) = str2double(parsed_name(6));
                i = i+1;
            else
                files(i) = [];
                filenames(i) = [];
                filenames_noext(i) = [];
                exts(i) = [];
            end
        else
            files(i) = [];
            filenames(i) = [];
            filenames_noext(i) = [];
            exts(i) = [];
        end
    end
    
    points = [x; y; zeros(1, length(x))]';

end


function id_dict = id2id_dict(id)

    id_dict = zeros(1,length(id));
    
    for i=1:length(id)
        if 0<=id(i) && id(i) <= 54
            id_dict(i) = id(i)-40+1;
        elseif 60<=id(i) && id(i) <= 64
            id_dict(i) = id(i)-45+1;
        else
            id_dict(i) = 0;
        end
    end
        
end
