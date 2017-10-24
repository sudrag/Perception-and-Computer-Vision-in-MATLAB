clear all
clc
close all

image_folder = 'D:\Spring 2017\Perception\Project 1\P1_submission\ColorSeg\TrainingSet\CroppedBuoys';
Gfilenames = dir(fullfile(image_folder, 'G_*.jpg'));
total_gimages = numel(Gfilenames);
GSamples = [];
R1=[];G1=[];B1=[];
%use 5 green images to train on the green data
for k=1:5
   full_name= fullfile(image_folder, Gfilenames(k).name);         %
    I = imread(full_name);        
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);
    [x,y]=size(R);
    inc=1;
    for i=1:x
        for j=1:y
    R1(inc,1)=R(i,j);
    G1(inc,1)=G(i,j);
    B1(inc,1)=B(i,j);
    inc=inc+1;
        end
    end
    GSamples = [GSamples; [R1 G1 B1]];
    
end
Rfilenames = dir(fullfile(image_folder, 'R_*.jpg'));
total_rimages = numel(Rfilenames);
RSamples = [];
R1=[];G1=[];B1=[];
for k=1:4
   full_name= fullfile(image_folder, Rfilenames(k).name);      
    I = imread(full_name);        
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);
    [x,y]=size(R);
    inc=1;
    for i=1:x
        for j=1:y
    R1(inc,1)=R(i,j);
    G1(inc,1)=G(i,j);
    B1(inc,1)=B(i,j);
    inc=inc+1;
        end
    end
    RSamples = [RSamples; [R1 G1 B1]];

end
Yfilenames = dir(fullfile(image_folder, 'Y_*.jpg'));
total_yimages = numel(Yfilenames);
YSamples = [];
R1=[];G1=[];B1=[];
for k=1:10
   full_name= fullfile(image_folder, Yfilenames(k).name);         
    I = imread(full_name);        
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);
    [x,y]=size(R);
    inc=1;
    for i=1:x
        for j=1:y
    R1(inc,1)=R(i,j);
    G1(inc,1)=G(i,j);
    B1(inc,1)=B(i,j);
    inc=inc+1;
        end
    end
    YSamples = [YSamples; [R1 G1 B1]];

end

%% Generate EM plots
%Green
[mug,sigmag]=Gmm(GSamples,5);
mug_r = mug(1,1);
mug_g = mug(1,2);
mug_b = mug(1,3);
sigmag_r = sqrt(sigmag(1,1,1));
sigmag_g = sqrt(sigmag(2,2,1));
sigmag_b = sqrt(sigmag(3,3,1));
%Yellow
[muy,sigmay]=Gmm(YSamples,5);
muy_r = muy(1,1);
muy_g = muy(1,2);
muy_b = muy(1,3);
sigmay_r = sqrt(sigmay(1,1,1));
sigmay_g = sqrt(sigmay(2,2,1));
sigmay_b = sqrt(sigmay(3,3,1));
%Red
[mur,sigmar]=Gmm(RSamples,5);
mur_r = mur(1,1);
mur_g = mur(1,2);
mur_b = mur(1,3);
sigmar_r = sqrt(sigmar(1,1,1));
sigmar_g = sqrt(sigmar(2,2,1));
sigmar_b = sqrt(sigmar(3,3,1));


%% Generate Video   
vid = VideoReader('detectbuoy.avi');   
NewVid = VideoWriter('GMMBuoyTest.mp4','MPEG-4');
NewVid.FrameRate=10;
open(NewVid);
for o=1:2:100
Tes=read(vid,o);

[finimg]=detectBuoys(Tes,mug,sigmag);
BWG2 = bwareaopen(finimg,50);
se2 = strel('disk',6);
BWG3 = imdilate(BWG2,se2);
BG = bwboundaries(BWG3);
figure(2),imshow(Tes)
hold on
%find centroid of region
GcenG = regionprops(BWG3,'Centroid');
%plot circle around centroid
if length(GcenG) > 0
gxg = GcenG(1).Centroid(1);
gyg = GcenG(1).Centroid(2);
scatter(gxg,gyg,250,'g','LineWidth',2)
end
% for  k =1:length(BG)
% boundary = BG{k};
% figure(2), plot(boundary(:,2), boundary(:,1), 'g', 'LineWidth', 2)
% hold on
% title(o)
% end 
[finimr]=detectBuoys(Tes,mur,sigmar);
BWR2 = bwareaopen(finimr,50);
se2 = strel('disk',6);
BWR3 = imdilate(BWR2,se2);
BR = bwboundaries(BWR3);
hold on
GcenR = regionprops(BWR3,'Centroid');
if length(GcenR) > 0
gxr = GcenR(1).Centroid(1);
gyr = GcenR(1).Centroid(2);
scatter(gxr,gyr,250,'r','LineWidth',2)
end
% for  k =1:length(BR)
% boundary = BR{k};
% figure(2), plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2)
% hold on
% title(o)
% end 
[finimy]=detectBuoys(Tes,muy,sigmay);
BWY2 = bwareaopen(finimy,50);
se2 = strel('disk',6);
BWY3 = imdilate(BWY2,se2);
BY = bwboundaries(BWY3);
hold on
GcenY = regionprops(BWY3,'Centroid');
if length(GcenY) > 0
gxy = GcenY(1).Centroid(1);
gyy = GcenY(1).Centroid(2);
scatter(gxy,gyy,250,'y','LineWidth',2)
end
% for  k =1:length(BY)
% boundary = BY{k};
% figure(2), plot(boundary(:,2), boundary(:,1), 'y', 'LineWidth', 2)
% hold on
% title(o)
% end 

f = getframe(gca);
% im = frame2im(f);
writeVideo(NewVid,f) %Add frame to video

end
close(NewVid)

