function [m_grid, detected] = findGridPoints(I, method, M_grid, control_points, grid_arrangement, file, cm2px_scale)
% FINDGRIDPOINTS Find grid points in image I
%
% Input:
%       - I: image
%       - method: {April, Checker, Rig}
%       - M_grid: grid points in 2D object-space (cm)
%       - control_points: grid delimiters in image space (pixels)
%       - grid_arrangement: [x-steps y-steps] grid steps along x, y axes
%       - file: filename of the image being processed
%       - cm2px_scale: dimension in cm of 1 pixel of the rectified image
%
% Output:
%       - m_grid: grid points detected in image space (pixels)
%
% Copyright by Andrea Fusiello, 2019
% 
% This work is licensed under the Creative Commons
% Attribution-NonCommercial-ShareAlike License.
% 
% To view a copy of this license, visit
% http://creativecommons.org/licenses/by-nc-sa/2.0/deed.en or send a
% letter to Creative Commons, 559 Nathan Abbott Way, Stanford,
% California 94305, USA.
%
% Modified by Claudio Verardo, Mattia Balutto (2021)
    
    if strcmp(method, 'April')
        % Call pyhton script and return the results (tags corners)
        [status, cmdout] = system(['./private/detectAprilTags.py  ' file.folder, '/', file.name]);
        m_grid  = str2num(cmdout)';
        if status>0
            error('something wrong with detectAprilTags.py');
        end
        
    else
        % Find grid points in image I that match template T close to
        % positions determined by M_grid transformed according to the
        % homography H
        
        % scale = .05; % dimension in cm of 1 pixel of the rectified image
        % The larger the scale, the smaller the rectified image, the faster
        % the detection, the lower the accuray
        scale = cm2px_scale;
        
        % scale M_grid (cm) to pixels
        M_grid = M_grid(1:2,:)./scale;
        % step size in pixels
        size_pix = round(M_grid(1,2) - M_grid(1,1));
        
        switch method
            
            case 'Checker'
                
                % initialization points  (match the ones clicked by the user)
                grid_xsteps = grid_arrangement(1);
                grid_ysteps = grid_arrangement(2);
                M4 = M_grid(:,[1, grid_xsteps, grid_xsteps*grid_ysteps, grid_xsteps*grid_ysteps-grid_xsteps+1]);
                % M4 = M_grid(:,[1,8,48,41]);
                
                % this is only for testing, normally the user should
                % provide input here
                % load([file.folder,'/m4']);  m4 = m4{i};
                m4 = control_points;
                % get points from the user anticlockwise fron the top-left
                % figure(1),  m4 = ginput';
                
                % corner template
                T = 255*ones(size_pix);
                T(1:ceil(size_pix/2), 1+ceil(size_pix/2):end ) = 0;
                T(1+ceil(size_pix/2):end, 1:ceil(size_pix/2) ) = 0;
                 
            case 'Rig'
                
                % initialization points  (match the ones clicked by the user)
                M4 = M_grid(:,[64,8,1,57]);
                
                % this is only for testing, normally the user should
                % provide input here
                % load([file.folder,'/m4']);  m4 = m4{i};
                m4 = control_points;
                % get points from the user in a given sequence
                % figure(1),  m4 = ginput';
                
                % corner template
                T = 255*ones(size_pix);
                T(1:ceil(size_pix/2) , 1:ceil(size_pix/2) ) = 0;
                
            otherwise
                error('unrecognised option\n');
                
        end
        
        % Homography from actual image to rectified image
        H = hom_lin(M4,m4);
        
        % set the bounding box of the rectified image
        bb = [1; 1; max(M_grid(1,:))+size_pix; max(M_grid(2,:))+size_pix];
        M_grid = M_grid - bb(1:2) + 1; % shift the origin  
        
        % recti\fy the image
        If = imwarp(I,@(x)htx(inv(H),x),bb);
      
        % template matching score
        S = real(tempMatching(If,T));
        % compute extrema of the score
        [val, idx] = extrema2(S);
        idx = idx(val>.7);
        [i,j] = ind2sub(size(S), idx');
        detected = [j;i];
        
        % solve assignment problem (hungarian method)
        cost = distmat(M_grid', detected');
        [assignment,~] = munkres(cost);
        detected = detected(:,assignment);
        
        % Alternative: find score max in a neighborhood of grid points
        %     ws = ceil(radius/3);
        %     detected = [];
        %     for M = ceil(M_grid)
        %         W = S(M(2)-ws:M(2)+ws, M(1)-ws:M(1)+ws);
        %       %  figure(10), imshow(W,[])
        %         [~,ind] = max(W(:)) ;
        %         [r,c] = ind2sub(size(W), ind);
        %         detected = [detected, [M(1)-ws+c; M(2)-ws+r]];
        %     end
        %
        
        % subpixel refinement
        for i = 1:length(detected)
            
            x = detected(2,i) + [-1, 0, 1];
            y = detected(1,i);
            [dx, ~] = subPix(x, S(x,y) );
            
            x = detected(2,i);
            y = detected(1,i) + [-1, 0, 1];
            [dy, ~] = subPix(y, S(x,y) );
                                  
            detected(:,i) = detected(:,i) + [dy;dx];
        end
        
        % show score in green overlaied onto the image
        figure, imshow(If,[], 'InitialMagnification', 'fit'), hold on;
        green = cat(3, zeros(size(S)),ones(size(S)), zeros(size(S)));
        hold on, h = imshow(green);  hold off
        set(h, 'AlphaData',S.^2), hold on
        plot(detected(1,:),detected(2,:),'+m','MarkerSize',15);
        plot(M_grid(1,:), M_grid(2,:),   'or','MarkerSize',15);
        title('Score')
        
        % bring back detected points to the original image
        m_grid = htx(inv(H),detected + bb(1:2) -1 );
    end
    
end
