function plot_aruco_markers(aruco_markers)
% PLOT_ARUCO_MARKERS Show the content of a set of Aruco markers.
%
%   PLOT_ARUCO_MARKERS(aruco_markers)
%
%   Input arguments:
%   ------------------
%   aruco_markers: cell array containing the Aruco markers
%
%   See also ARUCO_DETECTION
    
    n_aruco_markers = size(aruco_markers,1);
    
    figure;
    for i=1:n_aruco_markers
        subplot(1,n_aruco_markers,i)
        imshow(aruco_markers{i},'InitialMagnification','fit');
        title(sprintf('Aruco %d', i));
    end
    suptitle('Aruco Markers');

end