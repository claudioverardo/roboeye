function img = get_image(img_source)
% GET_IMAGE Acquire an image from camera or load an image from disk.
%
%   img = GET_IMAGE(img_source)
%
%   Input arguments:
%   ------------------
%   img_source:     webcam object or path to an image on disk
%
%   Output arguments:
%   ------------------
%   img:            image acquired from the camera or loaded from disk
%
%   See also WEBCAM

    if isa(img_source,'webcam') 
        img = snapshot(img_source);
        fprintf('Image acquired from camera\n');
    elseif isa(img_source,'char') || isa(img_source,'string') 
        img = imread(img_source); 
        fprintf('Image loaded\n%s\n', img_source);
    else
        error('Error, img_source not valid');
    end

end