function img = get_image(img_source)

    if isa(img_source,'webcam') 
        img = snapshot(img_source);
        fprintf('Image acquired\n');
    elseif isa(img_source,'char') || isa(img_source,'string') 
        img = imread(img_source); 
        fprintf('Image loaded\n');
    else
        error('Error, img_source not valid');
    end

end