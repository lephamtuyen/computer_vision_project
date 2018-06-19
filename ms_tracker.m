%%  Final project: Video Object Tracking
%   Course: Computer Vision
%   Author: Le Pham Tuyen
%   ID:     2014311082
%   Algorithm: Mean Shift
function ms_tracker
    %% Clean workspace, console window
    clear all;
    close all;
    clc;
    
    %% Initial parameters of mean shift
    % Number of time run mean shift
    maxIter = 30;            
    
    %% Enter the path to folder containing images sequence
    fprintf('----------------------------------------------------------------.\n');
    fprintf('Enter the path to the folder containing the images sequence.\n');
    fprintf('NOTICE: folder name must be placed in single quotes and with a slash (''/'' or ''\'') at the end.\n');
    fprintf('Example: ''../tracking_test/test1/''\n');
    path = input('Path: ');
    
    % Create object folder of images
    folder = dir(path);
    
    % Get all image file name
    allImagesName = {folder(~[folder.isdir]).name};
    
    % Read number of frame from image sequence
    nFrames = size(allImagesName, 2);
    
    %% Enter the index of started frame image which is used to get template of object
    startIdx = 0;
    % Re-enter if index is less than 0 or greater than number of images
    while (startIdx < 1 || startIdx >= nFrames)
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

    %% Get pdf of colors in object template
    pdf = get_pdf(double(startedFrame), template);
    
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
        
    %% Start tracking using Mean Shift
    for i = startIdx:nFrames
        
        % Read the ith frame
        filename = strcat(path, allImagesName{i});
        currFrame=imread(filename);
        
        % tracking target with mean shift
        [template, lost] = ms_tracking(double(currFrame), template, pdf, maxIter);
        
        % Stop tracking if we lost object
        if (lost == true)
            fprintf('Stop tracking because object goes outside the image\n');
            break;
        end
        
        % Draw bounding box around object
        currFrame = draw_bounding_box(template, currFrame, 2);
        
        % Display tracked frame image
        if i == startIdx
            handle_imshow=imshow(currFrame);
            title([num2str(i),'/',num2str(nFrames)]);
            hold on;
        else
            set(handle_imshow,'Cdata',currFrame);
            title([num2str(i),'/',num2str(nFrames)]);
            drawnow;
        end
        
        %% Save tracked frame
%         f = strcat(resultFolder, '/', int2str(i), '.png');
%         imwrite(currFrame, f);
    end
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
function template = get_template(frame_image)
    
    % Show the frame image
    imshow(frame_image);
    
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

%% function pdf_template = get_template_pdf(image_frame, template)
% Descriptioin: Get template of object from frame image
%
% Arguments:
%            image_frame    - image which is used to get object template
%            template       - template of image
%
% Returns:
%            pdf_template   - pdf of template object
% 
function pdf_template = get_template_pdf(image_frame, template)
    % Only use 4 bit to represent each color (R , G and B)
    pdf_template = zeros(1, 16^3);
    [height, width, ~] = size(image_frame);
    
    % Loop though the template and record distribution of colors
    for i=template.rmin:template.rmax
        for j=template.cmin:template.cmax
            if (i >= 1 && i <= height && j >= 1 && j <= width)
                % Reduce from 256 color to 16 color
                R=floor(image_frame(i,j,1)/16)+1;
                G=floor(image_frame(i,j,2)/16)+1;
                B=floor(image_frame(i,j,3)/16)+1;
                idx=(R-1)*256+(G-1)*16+B;
                pdf_template(idx) = pdf_template(idx)+1;
            end
        end
    end
    
    % Normalize
    pdf_template = pdf_template/sum(pdf_template);
end

%% function pdf_bg = get_background_pdf(frame_image, template)
% Descriptioin: Get template of object from frame image
%
% Arguments:
%            image_frame    - image which is used to get object template
%            template       - template of image
%
% Returns:
%            pdf_bg         - pdf of background around template
% 
function pdf_bg = get_background_pdf(frame_image, template)
    % Only use 4 bit to represent each color (R , G and B)
    pdf_bg = zeros(1, 16^3);
    [height, width, ~] = size(frame_image);
    
    % Get size of background (= 2 times size of template)
    rmin_bg=template.rmin-template.half_window(1);   %
    rmax_bg=template.rmax+template.half_window(1);   %
    cmin_bg=template.cmin-template.half_window(2);   %
    cmax_bg=template.cmax+template.half_window(2);   %
    
    % Loop though the background of template and record distribution of colors
    for i=rmin_bg:rmax_bg 
        for j=cmin_bg:cmax_bg 
            if (i >= 1 && i <= height && j >= 1 && j <= width)
                if (i < template.rmin || i > template.rmax || j < template.cmin || j > template.cmax)
                    % Reduce from 256 color to 16 color
                    R=floor(frame_image(i,j,1)/16)+1;
                    G=floor(frame_image(i,j,2)/16)+1;
                    B=floor(frame_image(i,j,3)/16)+1;
                    idx=(R-1)*256+(G-1)*16+B;
                    pdf_bg(idx)=pdf_bg(idx)+1;
                end
            end
        end
    end
    
    % Normalize
    pdf_bg = pdf_bg/sum(pdf_bg);
    
    % Get Background-Weighted Histogram
    T = find(pdf_bg~=0);
    % minimal non-zero value 
    non_zero_min = min(pdf_bg(T));
    
    % Background-Weighted Histogram
    % pdf_bg = min(1, non_zero_min/pdf_bg(i))
    for i=1:4096
        if pdf_bg(i)~=0
            pdf_bg(i)=non_zero_min/pdf_bg(i);
        else
            pdf_bg(i)=1;
        end
    end
end

%% function pdf = get_pdf(image,template)
% Descriptioin: Get pdf of object template using Background-Weighted Histogram 
%
% Arguments:
%            frame_image- image which is used to get object template
%            template   - image which is used to get object template
%
% Returns:
%            pdf_template   - pdf of object template
% 
function pdf_template = get_pdf(frame_image, template)
    % Get pdf of template object
    pdf_template = get_template_pdf(frame_image, template);
    % Get pdf of background around the template
    pdf_bg = get_background_pdf(frame_image, template);
    
    % New Pdf using Background-Weighted Histogram
    pdf_template = pdf_template.*pdf_bg/(pdf_template*pdf_bg');
end

%% new_template = ms_tracking(image, template, pdf, minDist ,max_iter)
% Descriptioin: Mean shift algorithm
%
% Arguments:
%            frame_image    - image which is used to get object template
%            template       - image which is used to get object template
%            pdf            - pdf of template
%            ms_threshold   - mean shift threshold
%            max_iter       - maxium iteration
%
% Returns:
%            new_template   - new position of template after mean shift
% 
function [new_template, lost] = ms_tracking(frame_image, template, pdf ,max_iter)
    new_template = template;
    
    % flag to indicate object still in track or not
    lost = false;
    
    % Mean shift distance
    ms = inf;
    
    % Counter number of time to run mean shift
    iter=0;
    old_center = new_template.center;
    
    height = size(frame_image,1);
    width = size(frame_image,2);
    
    % Loop until max_iter
    while iter < max_iter
        
        % Get new pdf of template in new position
        currPdf = get_template_pdf(frame_image, new_template);
        
        % Bhattacharyya similarity 
        n=1;
        for i=new_template.rmin:new_template.rmax
            for j=new_template.cmin:new_template.cmax
                % Only get pixel inside the frame image
                if (i >= 1 && i <= height && j >= 1 && j <= width)
                    % Get color index in pdf
                    R = floor(frame_image(i,j,1)/16)+1;
                    G = floor(frame_image(i,j,2)/16)+1;
                    B = floor(frame_image(i,j,3)/16)+1;
                    idx = (R-1)*256+(G-1)*16+B;
                    
                    % Get similarity point location
                    xi(1,n)=i;
                    xi(2,n)=j;
                    
                    % Calculate the weight i
                    w_i(n)=sqrt(pdf(idx)/currPdf(idx));
                    n=n+1;
                end
            end
        end
        
        % Calculate te new location of template center
        new_center = (xi*w_i'/sum(w_i))';
        
        % Check mean shift is convergence or not
        new_ms = sqrt(sum((new_center-old_center).^2)); 

        % Return if mean shift is not convergence
        if (new_ms > ms)
            break;
        else
            ms = new_ms;
        end
        
        % Update center of template
        new_template.center = floor(new_center);
        old_center = new_template.center;
        new_template.rmin = new_template.center(1)-new_template.half_window(1);
        new_template.rmax = new_template.center(1)+new_template.half_window(1);
        new_template.cmin = new_template.center(2)-new_template.half_window(2);
        new_template.cmax = new_template.center(2)+new_template.half_window(2);
        
        % Stop tracking if object go outside the image
        if (new_template.rmin < -new_template.half_window(1)/2 ...
                || (new_template.rmax - height) > new_template.half_window(1)/2 ...
                || new_template.cmin < -new_template.half_window(2)/2 ...
                || (new_template.cmax - width) > new_template.half_window(2)/2)
            lost = true;
            break;
        end
        
        % Process the next iteration
        iter = iter + 1;                   
    end
end