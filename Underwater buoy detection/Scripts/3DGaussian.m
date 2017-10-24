%% 3D Gaussian


%3D Gaussian Estimation
[muR, covR, muG, covG, muY, covY] = Estimation3D();

% 3D Gaussian Buoy Detection
vid = VideoReader('detectbuoy.avi');
nFrames = vid.NumberOfFrames();
v3D = VideoWriter('buoy_3D','MPEG-4');
v3D.FrameRate = 10;
open(v3D);
for f = 1:2:80 %99
    %full_name= fullfile(imagepath, filename(f).name);
    %img = imread(full_name);
    img = read(vid,f);
    
    [w, d, c] = size(img);
    red = img(:,:,1);
    green = img(:,:,2);
    blue = img(:,:,3);
    yellow = (double(red) + double(green))/2;
    
    %% Probability map of the image
    %RED
    for i = 1:w
        for j = 1:d
            x = [double(red(i,j)), double(green(i,j)), double(blue(i,j))];
            p3dR(i,j) = mvnpdf(x, muR, covR);
        end
    end
%   figure(3), imshow(prob3dR);

    %YELLOW
    for i = 1:w
        for j = 1:d
            x = [double(red(i,j)), double(green(i,j)), double(blue(i,j))];
            p3dY(i,j) = mvnpdf(x, muY, covY);
        end
    end
%     figure(3), imshow(prob3dY);

    %GREEN
    for i = 1:w
        for j = 1:d
            x = [double(red(i,j)), double(green(i,j)), double(blue(i,j))];
            p3dG(i,j) = mvnpdf(x, muG, covG);
        end
    end
%     figure(3), imshow(prob3dG);
    
    %% Normalizing
    output_range =  255 - 0;
    
    %RED
    pmax3dR = max(p3dR(:));
    input_rangeR = pmax3dR - 0;
    MapR = p3dR*(output_range/(input_rangeR));
    new_MapR = MapR;
%     figure(4), imshow(MapRnew)
    
    %YELLOW
    pmax3dY = max(p3dY(:));
    input_rangeY = pmax3dY - 0;
    MapY = p3dY*(output_range/(input_rangeY));
    new_MapY = MapY;
%     figure(5), imshow(MapYnew)
    
    %GREEN
    pmax3dG = max(p3dG(:));
    input_rangeG = pmax3dG - 0;
    MapG = p3dG*(output_range/(input_rangeG));
    new_MapG = MapG;
%     figure(4), imshow(MapGnew)
    
    %% Binarization
    %RED
    indexR = (new_MapR < 249);
    new_MapR(indexR) = 0;
    se1 = strel('disk', 10);
    new_MapR = imdilate(new_MapR,se1);
%     figure(7), imshow(MapRnew)

    %YELLOW
    indexY = (new_MapY < 249);
    indexYr = (new_MapR > 49);
    new_MapY(indexY) = 0;
    new_MapY(indexYr) = 0;
    se1 = strel('disk', 10);
    new_MapY = imdilate(new_MapY,se1);
%     figure(8), imshow(MapYnew)

    %GREEN
    indexG = (new_MapG < 250);
    new_MapG(indexG) = 0;
    se1 = strel('disk', 1);
    se2 = strel('disk', 10);
%     MapGnew = imerode(MapGnew,se1);
    new_MapG = imdilate(new_MapG,se2);
    %figure(9), imshow(MapGnew)
    
    
    %% Get the buoy segmented
    figure(6), imshow(img)
    hold on
    %RED
    Rcheck = 0;
    new_MapR = im2bw(new_MapR);
    cent_R = regionprops(new_MapR,'Centroid');
    if (length(cent_R) > 0)
        rx = cent_R(1).Centroid(1);%Find center of blob we made using pixel. Blob isnt perfect representation of Buoy
        ry = cent_R(1).Centroid(2);
        scatter(rx, ry, 300,'r','LineWidth', 2);
    end

    
    %YELLOW
    Ycheck = 0;
    new_MapY = im2bw(new_MapY);
    cent_Y = regionprops(new_MapY,'Centroid');
    if (length(cent_Y) > 0)
        yx = cent_Y(1).Centroid(1);%Find center of blob we made using pixel. Blob isnt perfect representation of Buoy
        yy = cent_Y(1).Centroid(2);
        scatter(yx, yy,300, 'y','LineWidth', 2);
    end

    
    %%GREEN
    Gcheck = 0;
    new_MapG = im2bw(new_MapG);
    cent_G = regionprops(new_MapG,'Centroid');
    if (length(cent_G) > 0 & f<46)
        gx = cent_G(1).Centroid(1);%Find center of blob we made using pixel. Blob isnt perfect representation of Buoy
        gy = cent_G(1).Centroid(2);
        scatter(gx, gy, 300,'g','LineWidth', 2);
    end

    
    %Get one only binary image
    binary = new_MapR | new_MapY | new_MapG;
%     figure(8), imshow(binary)
    binpath = strcat('C:\Users\Aaron Sirken\Documents\MATLAB\Perception\Hw1\GMM',num2str(f,'%03i'),'.jpg');
    %imwrite(binary,binpath);
    

    
    frame = getframe(gca);
    im = frame2im(frame);
    impath = strcat('C:\Users\Aaron Sirken\Documents\MATLAB\Perception\Hw1\GMM_',num2str(f,'%03i'),'.jpg');
    %imwrite(im,impath);
    writeVideo(v3D,frame);
    f
end
close(v3D)
