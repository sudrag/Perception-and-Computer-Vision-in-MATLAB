clc
close all
clear all

vid = VideoReader('./P2_Submission/LaneDetection/Input/project_video.mp4');
nFrames = vid.NumberOfFrames();

v = VideoWriter('lane_detection','MPEG-4');
v.FrameRate = 25;
open(v);

for f = 220 : 480
    f
    img = read(vid,f);
    
    %% Denoise the image
    % Apply a Gaussian filter
    mu = 5;
    sigma = 5;
    index = -floor(mu/2) : floor(mu/2);
    [X Y] = meshgrid(index, index);
    G = exp(-(X.^2 + Y.^2) / (2*sigma*sigma));
    G = G / sum(G(:));
    img_filtered = imfilter(img, G);
%     img_filtered = imgaussfilt(img, 0.8);
%     figure(1), imshow(img_filtered);
    
    %% Edge Detection
%    gray=im2bw(img_filtered,0.8); 
   gray = rgb2gray(img);
   gray= im2bw(gray,0.64);
   imshow(gray);
%     figure(2), imshow(gray)
    bin = imfilter(gray, [-1 0 1], 'replicate','corr');
%     bin = edge(gray,'Canny', [0.3, 0.5]);
%     figure(3), imshow(bin)    
    
%     img(I < 0) = 0;
%     img(I > 1) = 1;
%     th = graythresh(bin); % compute threshold
%     bin = im2bw(bin, 0.12);
%     figure(4), imshow(bin)
    
    %% Crop ROI
    x = [210 550 717 1280];
    y = [720 450 450 720];
    poly_top = y(2) + 20;
    mask = poly2mask(x,y, 720, 1280);
    
    crop = immultiply(bin,mask);
%     figure(5), imshow(crop)
        
    %% Hough Transform
    [H, theta, rho] = hough(crop);
    
    %Hough Peaks
    P = houghpeaks(H,10,'threshold',ceil(0.1*max(H(:))));
    
    %Hough Lines
    line = houghlines(crop,theta,rho,P,'MinLength',15);
%     figure(6), imshow(crop) 
%     hold on
    for k = 1:length(line)
       xy = [line(k).point1; line(k).point2];
%        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','red');
       
       % Plot beginning and end of lines
%        plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','green');
%        plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','green');
    end
    
    %% Lane Lines
    %Get the slope of each of the lines that could be part of the lane
    slope_thresh = 0.3;
    i = 1;
    for k = 1:length(line)
        ini = line(k).point1;
        fin = line(k).point2;
        if fin(1) - ini(1) == 0
           slope = 999; 
        else
           slope = (fin(2) - ini(2))/(fin(1) - ini(1));
        end
        if abs(slope) > slope_thresh
            slopes(i) = slope;
            lines(i) = line(k);
            i = i + 1;
        end
    end
    
    %Split the lines into right or left lines
    img_size = size(img);
    center = img_size(2)/2;
    i = 1; j = 1;
    for k = 1:length(lines)
        ini = lines(k).point1;
        fin = lines(k).point2;
        if slopes(k) > 0 && fin(1) > center && ini(1) > center
            right_lines(i) = lines(k);
            right_flag = 1;
            i = i + 1;
        elseif slopes(k) < 0 && fin(1) < center && ini(1) < center
            left_lines(j) = lines(k);
            left_flag = 1;
            j = j + 1;
        else
            right_flag = 0;
            left_flag = 0;
        end
    end
    
    %Linear regression to fit a polynomial for right/left line
    if (right_flag == 0 || left_flag == 0)
        continue
    end   
    %Right line
    i = 1;
    if right_flag == 1
        for k = 1:length(right_lines)
            ini = right_lines(k).point1;
            fin = right_lines(k).point2;

            right_x(i) = ini(1);
            right_y(i) = ini(2);
            i = i + 1;

            right_x(i) = fin(1);
            right_y(i) = fin(2);
            i = i + 1;
        end
        if length(right_x) > 0
            % y = m*x + b
            pol = polyfit(right_x, right_y, 1);
            right_m = pol(1);
            right_b = pol(2);
        else
            right_m = 1;
            right_b = 1;
        end
    end
    
    %Left Lane
    i = 1;
    if left_flag == 1
        for k = 1:length(left_lines)
            ini = left_lines(k).point1;
            fin = left_lines(k).point2;

            left_x(i) = ini(1);
            left_y(i) = ini(2);
            i = i + 1;

            left_x(i) = fin(1);
            left_y(i) = fin(2);
            i = i + 1;
        end
        if length(left_x) > 0
            % y = m*x + b
            pol = polyfit(left_x, left_y, 1);
            left_m = pol(1);
            left_b = pol(2);
        else
            left_m = 1;
            left_b = 1;
        end
    end
    
    %Once we have the lines, find their endpoints with equation of a line
    % x = (y - b) / m
    ini_y = img_size(1);
    fin_y = poly_top;
    
    right_ini_x = (ini_y - right_b) / right_m;
    right_fin_x = (fin_y - right_b) / right_m;
    
    left_ini_x = (ini_y - left_b) / left_m;
    left_fin_x = (fin_y - left_b) / left_m;
    
    
    %% Plotting
    % Draw Polygon for lane (Stackoverflow...)
    pt1 = [left_ini_x, ini_y];
    pt2 = [left_fin_x, fin_y];
    pt3 = [right_fin_x, fin_y];
    pt4 = [right_ini_x, ini_y];
    pt_x = [pt1(1) pt2(1) pt3(1) pt4(1)];
    pt_y = [pt1(2) pt2(2) pt3(2) pt4(2)];
    BW = poly2mask(pt_x, pt_y, 720, 1280);
    clr = [255 0 0];            % red color
    a = 0.3;                    % blending factor
    z = false(size(BW));
    mask = cat(3,BW,z,z); img(mask) = a*clr(1) + (1-a)*img(mask);
    mask = cat(3,z,BW,z); img(mask) = a*clr(2) + (1-a)*img(mask);
    mask = cat(3,z,z,BW); img(mask) = a*clr(3) + (1-a)*img(mask);
    figure(10), imshow(img)
    hold on
    %Plot the lines
    plot([left_ini_x, left_fin_x],[ini_y, fin_y],'LineWidth',5,'Color','yellow');
    plot([right_ini_x, right_fin_x],[ini_y, fin_y],'LineWidth',5,'Color','yellow');
    
    %% Predict the turn
    %Get the vanishing point
    vanish_x = (right_b - left_b) / (left_m - right_m)-25; 
    vanish_y = right_m * vanish_x + right_b;
    thr_van = 5;
    if (vanish_x < (center - thr_van))
       text_str = 'Left';
       t = text(550,600,text_str,'Color','k','FontSize',30); 
    elseif (vanish_x > (center + thr_van))
       text_str = 'Right';
       t = text(550,600,text_str,'Color','k','FontSize',30);
    elseif ((center - thr_van) <= vanish_x <= (center + thr_van))
       text_str = 'Straight';
       t = text(550,600,text_str,'Color','k','FontSize',30);
    end
    
    text_str2 = ['VP: ' num2str(vanish_x)];
    t = text(30,30,text_str2,'Color','g','FontSize',12);
    
    %Fit spline in the lane
    x_2 = left_x;
    y_2 = left_y;
    p_2 = polyfit(x_2, y_2, 2);
    t1 = left_ini_x:1:left_fin_x;
    yt = p_2(1)*(t1.^2) + p_2(2)*t1 + p_2(3);
%     plot(t1,yt,'LineWidth',5,'Color','yellow');
    
    x_3 = [right_ini_x right_fin_x vanish_x];
    y_3 = [ini_y fin_y vanish_y];
    p_3 = polyfit(x_3, y_3, 2);
    t3 = right_fin_x:1:right_ini_x;
    yt3 = p_3(1)*(t3.^2) + p_3(2)*t3 + p_3(3);
%     plot(t3,yt3,'LineWidth',5,'Color','yellow');
   
    
    frame = getframe(gca);
    writeVideo(v,frame);
    
    clear lines;
    clear slopes;
    clear right_lines;
    clear left_lines;
    clear right_x;
    clear right_y;
    clear left_x;
    clear left_y;
  
    pause(0.001)
end

close(v)