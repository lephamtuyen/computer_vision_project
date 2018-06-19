%%  Final project: Video Object Tracking
%   Course: Computer Vision
%   Author: Le Pham Tuyen
%   ID:     2014311082
%   Algorithm: Lucas Kanade
function lk_tracker
    %% Clean workspace, console window
    clear all;
    close all;
    clc;
    
    %% Initial parameters
    % Number of time run mean shift
    maxIter = 20;
    % Sigma value of standard Gaussian
    sig = 3;
    % Gaussian filter for image
    [kernelx, kernely] = gaussian_kernel(sig);
    % Offset threshold of Lucas Kanade
    offsetThresh = 0.01; % accuracty desired in terms of pixel-width
    
    
    %% Enter the path to folder containing images sequence
    fprintf('----------------------------------------------------------------.\n');
    fprintf('Enter the path to the folder containing the images sequence.\n');
    fprintf('NOTICE: folder name must be placed in single quotes and with a slash (''/'' or ''\'') at the end.\n');
    fprintf('Example: ''../tracking_test/test5/''\n');
    path = input('Path: ');
    
    % Create object folder of images
    folder = dir(path);
    
    % Get all image file name
    allImagesName = {folder(~[folder.isdir]).name};
    
    % Read number of frame from image sequence
    nFrames = size(allImagesName, 2);
    
    %% Enter the index start image which is used to get template of object
    startIdx = 0;
    % Re-enter if index is less than 0 or greater than number of images
    while (startIdx < 1 || startIdx > nFrames)
        fprintf('----------------------------------------------------------------.\n');
        fprintf('There have %d frames.\n', nFrames);
        fprintf('Enter index of start frame which is used to get object template.\n');
        fprintf('Start frame index must be greater than 0 and less than %d.\n', nFrames);
        startIdx = input('Start frame index: ');
    end
    
    %% Get template of object
    % Read start frame image
    filename = strcat(path, allImagesName{startIdx});
    startedFrame = imread(filename);
    
    % Get template of object at start frame
    template = get_template(startedFrame);
    
    %% Create folder of tracked frames (Only for statistical purpose)
%     resultFolder = strcat(path, 'result');
%     i = 1;
%     while isdir(resultFolder)
%         % Remove folder
%         resultFolder = strcat(path, 'result', int2str(i));
%         i = i + 1;
%     end
%     
%     % Create new folder
%     mkdir(resultFolder);
    
    %% Start tracking using Lucas Kanade
    prevFrame = double(rgb2gray(startedFrame));
    
    % Initial parameter for interpolate warping
    [height,width] = size(prevFrame);
    [X,Y] = meshgrid(1:width,1:height);
    
    % Loop though frame image
    for i = startIdx:nFrames
        
        % Read the ith frame
        filename = strcat(path, allImagesName{i});
        im_to_show = imread(filename);
        
        % Current frame image
        currFrame = double(rgb2gray(im_to_show));
        
        % Initial offset of object from previous image to current image
        % prev_frame + [u, v] = curr_frame
        u = 0;
        v = 0;
        
        % Adjust bounding box to make it do not go out the frame size
        [rmin, rmax, cmin, cmax] = adjust_bounding_box(template, currFrame);
        
        % Find matrix C
        Ix = conv2(prevFrame, kernelx,'same');
        Iy = conv2(prevFrame, kernely,'same');
        
        Ix2 = sum(sum(Ix(rmin:rmax, cmin:cmax).^2));
        Iy2 = sum(sum(Iy(rmin:rmax, cmin:cmax).^2));
        Ixy = sum(sum(Ix(rmin:rmax, cmin:cmax).*Iy(rmin:rmax, cmin:cmax)));
        
        % C = [Ix^2 Ix*Iy; Ix*Iy Iy^2]
        C = [Ix2 Ixy; Ixy Iy2];
        
        for j=1:maxIter
            
            % Warping current frame in coordinate system of previous image
            WCurrFrame = interp2(currFrame,X+u,Y+v);
            % Set NaN pixels to 0
            pixels = find(isnan(WCurrFrame));
            WCurrFrame(pixels) = 0;
            
            % Find matrix D
            It = WCurrFrame - prevFrame;
            Itx = sum(sum(It(rmin:rmax, cmin:cmax).*Ix(rmin:rmax, cmin:cmax)));
            Ity = sum(sum(It(rmin:rmax, cmin:cmax).*Iy(rmin:rmax, cmin:cmax)));
            
            % D = [It*Ix; It*Iy]
            D = [Itx; Ity];
            
            % Solve Lucas Kanade equation to find the offset
            % [u, v] = inv(C)*D
            uv = C\D;
            
            % Update the new offset
            du = uv(1); dv = uv(2);
            u = u - du;  v = v - dv;
            
            % Escape loop if offset is less than a threshold
            if ((abs(du) <= offsetThresh && abs(dv) <= offsetThresh))
                break;
            end
        end
        
        % Update center of template
        u = round(u);
        v = round(v);
        template.center = template.center + [u, v];
        template.rmin = template.rmin + v;
        template.rmax = template.rmax + v;
        template.cmin = template.cmin + u;
        template.cmax = template.cmax + u;
        
        % Stop tracking if object go outside the image
        if (template.rmin < 0 ...
                || template.rmax > height ...
                || template.cmin < 0 ...
                || template.cmax > width)
            fprintf('Stop tracking because object goes outside the image\n');
            break;
        end
        
        % Update previous frame image
        prevFrame = currFrame;
        
        % Draw bounding box around object
        im_to_show = draw_bounding_box(template, im_to_show, 2);
        
        % Display tracking image sequence
        if i == startIdx
            handleShow=imshow(im_to_show);
            title([num2str(i),'/',num2str(nFrames)]);
            hold on;
        else
            set(handleShow,'Cdata',im_to_show);
            title([num2str(i),'/',num2str(nFrames)]);
            drawnow;
        end
        
        %% Save tracked frame
%         f = strcat(resultFolder, '/', int2str(i), '.png');
%         imwrite(im_to_show, f);
    end
end

%% [kernelx, kernely] = gaussian_kernel(sig)
% Descriptioin: Get kernel of gaussian by x and y
%
% Arguments:
%            sig            - sigma value
%
% Returns:
%            kernelX        - kernel of gaussian by x
%            kernelY        - kernel of gaussian by y
%
function [kernelX, kernelY] = gaussian_kernel(sig)
    % Number of x value = 3 times of sig
    x = floor(-3*sig):ceil(3*sig);
    
    % Get gaussian value
    G = exp(-0.5*x.^2/sig^2);
    G = G/sum(G);
    
    % Get first derivetive of gaussian
    dG = -x.*G/sig^2;
    
    % Get kernel
    kernelX = G'*dG;
    kernelY = kernelX';
end

%% [rmin, rmax, cmin, cmax] = adjust_bounding_box(template, frame_image)
% Descriptioin: Adjust bounding box to make it do not go out the frame size
%
% Arguments:
%            template           - template of object
%            frame_image      	- current image
%
% Returns:
%            rmin     - min row
%            rmax     - max row
%            cmin     - min column
%            cmax     - max column
%
function [rmin, rmax, cmin, cmax] = adjust_bounding_box(template, frame_image)
    % Get frame image size
    [frame_height, frame_width, ~] = size(frame_image);
    
    % Adjust row min
    rmin = template.rmin;
    if rmin < 1
        rmin = 1;
    end
    
    % Adjust row max
    rmax = template.rmax;
    if rmax > frame_height;
        rmax = frame_height;
    end
    
    % Adjust column min
    cmin = template.cmin;
    if cmin < 1
        cmin = 1;
    end
    
    % Adjust column max
    cmax = template.cmax;
    if cmax > frame_width
        cmax = frame_width;
    end
end
%% modified_frame = draw_bounding_box(template, frame_image, border_size)
% Descriptioin: Draw a bounding box around tracked object
%
% Arguments:
%            template           - template of object
%            frame_image      	- current image
%            border_size        - thick of border around object
%            u                  - offset x of center
%            v                  - offset y of center
%
% Returns:
%            modified_frame     - image after adding bounding box
%
function modified_frame = draw_bounding_box(template, frame_image, thick_size)
    modified_frame = frame_image;
    
    % Adjust bounding box to make it do not go out the frame size
    [rmin, rmax, cmin, cmax] = adjust_bounding_box(template, frame_image);
    
    if ((rmax - rmin) < thick_size || (cmax - cmin) < thick_size)
        return;
    end
    
    % Draw bounding box (rectangle with border size = thick) with red
    % color
    modified_frame(rmin:rmax, cmin:cmin+thick_size,:) = 0;
    modified_frame(rmin:rmax, cmin:cmin+thick_size,1) = 255;
    modified_frame(rmin:rmax, cmax-thick_size:cmax,:) = 0;
    modified_frame(rmin:rmax, cmax-thick_size:cmax,1) = 255;
    
    modified_frame(rmin:rmin+thick_size, cmin:cmax,:) = 0;
    modified_frame(rmin:rmin+thick_size, cmin:cmax,1) = 255;
    modified_frame(rmax-thick_size:rmax, cmin:cmax,:) = 0;
    modified_frame(rmax-thick_size:rmax, cmin:cmax,1) = 255;
end

%% function template = get_template(frame)
% Descriptioin: Get template of object from frame image
%
% Arguments:
%            modified_frame  	- image which is used to get object template
%
% Returns:
%            template   - template of object
%                           + Center
%                           + Extrema value of rectangle around object
%                           + half size of object
%
function template = get_template(frame)
    
    % Show the frame image
    imshow(frame);
    
    % Select bounding box of object using mouse
    rect = getrect;
    rect = round(rect);
    
    % Get coordinate of bounding box
    template.cmin = rect(1);
    template.cmax = rect(1)+rect(3)-1;
    template.rmin = rect(2);
    template.rmax = rect(2)+rect(4)-1;
    
    template.center(1) = round((template.rmin+template.rmax)/2);
    template.center(2) = round((template.cmin+template.cmax)/2);
    
    template.half_window(1) = round((template.rmax-template.rmin)/2);
    template.half_window(2) = round((template.cmax-template.cmin)/2);
end
