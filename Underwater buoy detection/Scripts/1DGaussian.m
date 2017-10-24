clear all
clc
vid = VideoReader('detectbuoy.avi');
NewVid = VideoWriter('1D-GaussianBuoyTest.mp4','MPEG-4');
NewVid.FrameRate=15;
open(NewVid);

for n = 1:120
frame = read(vid,n);
%Getting the channels
redChannel = frame(:, :, 1);
greenChannel = frame(:, :, 2);
blueChannel = frame(:, :, 3);


[s1 s2] = size(redChannel);

%MEAN AND STND DEV
%calculated mean and std dev values for red buoy.
%All means were calculated for 5 test images on a separate script
%The values were then copied here
uR = 225.0567;
stdR = 26.6450;

%calculated mean and std dev values for yellow buoy
%green channel for Y buoy
uYg = 166.5914;
stdYg = 23.29125;
%red Channel for Y buoy
uYr = 105.0428;
stdYr = 40.7458;
%blue channel for Y buoy
uYb = 131.5837;
stdYb = 11.6167418;

%Calculated mean and std dev values for green buoy
%Green channel for G buoy
uG = 224.5001;
stdG = 20.4531;
%Red channel for G buoy
uGred = 141.8698;
stdGred = 25.1141;
%Blue channel for G buoy
uGblue = 129.2225;
stdGblue = 11.5068;

%Finding PDF
%Only one channel was used for red and 3 for green and yellow
ProbR = normcdf(double(redChannel),uR,stdR);

ProbYr = normcdf(double(redChannel),uYr,stdYr);
ProbYb = normcdf(double(blueChannel),uYb,stdYb);
ProbYg = normcdf(double(greenChannel),uYg,stdYg);

ProbG = normcdf(double(greenChannel),uG,stdG);
ProbGr= normcdf(double(redChannel),uGred,stdGred);
ProbGb= normcdf(double(blueChannel),uGblue,stdGblue);

%Creating Binary Image
%The probabilites were thresholded by trial and error for each buoy
%red
BW = im2bw(frame);
threshold = .8;
indexW = (ProbR >= threshold);
indexB = (ProbR < threshold);
BW(indexW) = 1;
BW(indexB) = 0;

%yellow
%Three probability thresholds were found were yellow and green and
%applied to the entire image
BWY = im2bw(frame);
for i =1:s1
    for o = 1:s2
        if (ProbYr(i,o) >= .995 &&  ProbYg(i,o) > .983 && ProbYb(i,o) < .2 )
            BWY(i,o) = 1;
        else
            BWY(i,o) = 0;
        end
    end
end

%green
BWG = im2bw(frame);

for i =1:s1
    for o = 1:s2
        if (ProbG(i,o) >= 0.8 && ProbR(i,o) <.3 && ProbGb(i,o) < .999)
            BWG(i,o) = 1;
        else
            BWG(i,o) = 0;
        end
    end
end
%Adjusting Binary image
BW2 = bwareaopen(BW,50);
se1 = strel('disk', 6);
BW3 = imdilate(BW2,se1);
imshow(BW3)

%yellow
BWY2 = bwareaopen(BWY,50);
se2 = strel('disk',6);
BWY3 = imdilate(BWY2,se2);
imshow(BWY3)

%green
BW2G = bwareaopen(BWG,10);
se3 = strel('disk', 6);
BW3G = imdilate(BW2G,se3);

imshow(frame)

hold on
%Using region props to define the centroid and circles
centroidloc = regionprops(BW3, 'BoundingBox', 'Centroid');
if (size(centroidloc) > 0)
rx = centroidloc(1).Centroid(1);%circle center is centroid of blob
ry = centroidloc(1).Centroid(2);
% rcircSize = centroidloc(1).BoundingBox(3)*centroidloc(1).BoundingBox(4);%circle area is area of box
scatter(rx,ry,200,'r','LineWidth', 2)%plot circle
end
%Draw Contours around found buoy. Show frame, then draw border of white
%blob in binary image
%figure
%red
%Try to draw countours on edges of blob
% B = bwboundaries(BW3);
% for  k =1:length(B)
% boundary = B{k};
% plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2)
% title(n)
% end 

%yellow
centroidlocY = regionprops(BWY3, 'BoundingBox', 'Centroid');
if (size(centroidlocY) > 0)
yx = centroidlocY(1).Centroid(1);%circle center is centroid of blob
yy = centroidlocY(1).Centroid(2);
% ycircSize = centroidlocY(1).BoundingBox(3)*centroidlocY(1).BoundingBox(4);%circle area is area of box
scatter(yx,yy,200,'y','LineWidth', 2)%plot circle
end
hold on
%Try to draw countours on edges of blob
% BY = bwboundaries(BWY3);
% for  k =1:length(BY)
% boundary = BY{k};
% plot(boundary(:,2), boundary(:,1), 'y', 'LineWidth', 2)
% title(n)
% end 

%green
centroidlocG = regionprops(BW3G, 'BoundingBox', 'Centroid');
if (size(centroidlocG) > 0)
gx = centroidlocG(1).Centroid(1);%circle center is centroid of blob
gy = centroidlocG(1).Centroid(2);
% gcircSize = centroidlocG(1).BoundingBox(3)*centroidlocG(1).BoundingBox(4);%circle area is area of box
scatter(gx,gy,200,'g','LineWidth', 2)%plot circle
end
hold on
%Try to draw countours on edges of blob
% BG = bwboundaries(BW3G);
% for  k =1:length(BG)
% boundary = BG{k};
% plot(boundary(:,2), boundary(:,1), 'g', 'LineWidth', 2)
% title(n)
% end 

f = getframe(gca);
im = frame2im(f);
writeVideo(NewVid,im) %Add frame to video
end
close(NewVid);

