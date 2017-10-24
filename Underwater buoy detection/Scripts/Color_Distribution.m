function [RedBuoySamples, YellowBuoySamples, GreenBuoySamples] = Color_Distribution()

close all
clc 

%% Color Distribution for each of the buoys
imagepath = './P1_Submission/ColorSeg/Images/TrainingSet/CroppedBuoys';
%filenames = dir(fullfile(imagepath, '*.jpg'));
fileGreen = dir(fullfile(imagepath, '*G*.jpg'));%Contains all files with 'green' in it and is a jpg
fileRed = dir(fullfile(imagepath, '*R*.jpg'));
fileYellow = dir(fullfile(imagepath, '*Y*.jpg'));

%% RED BUOY
RedR = [];
GreenR = [];
BlueR = [];
YellowR = [];
for k=1:50
   full_name= fullfile(imagepath, fileRed(k).name);
   I = imread(full_name);
%    figure, imshow(I);
   red = I(:, :, 1);
   green = I(:, :, 2);
   blue = I(:, :, 3);
   yellow = (double(red) + double(green))/2;
   [a b] = size(red);
   for i = 1:a
        for j = 1:b
                RedR(end+1) = red(i,j);
                GreenR(end+1) = green(i,j);
                BlueR(end+1) = blue(i,j);
                YellowR(end+1) = yellow(i,j);
        end
   end
end
RedR = RedR';
GreenR = GreenR';
BlueR = BlueR';
YellowR = YellowR';

figure(1), scatter3(RedR,GreenR,BlueR,5,'r.')
xlabel('Red')
ylabel('Green')
zlabel('Blue')

RedBuoySamples = [RedR, GreenR, BlueR];

%% YELLOW BUOY
RedY = [];
GreenY = [];
BlueY = [];
YellowY = [];
for k=1:50
   full_name= fullfile(imagepath, fileYellow(k).name);
   I = imread(full_name);
   red = I(:, :, 1);
   green = I(:, :, 2);
   blue = I(:, :, 3);
   yellow = (double(red) + double(green))/2;
   [a b] = size(red);
   for i = 1:a
        for j = 1:b
                RedY(end+1) = red(i,j);
                GreenY(end+1) = green(i,j);
                BlueY(end+1) = blue(i,j);
                YellowY(end+1) = yellow(i,j);
        end
   end
end
RedY = RedY';
GreenY = GreenY';
BlueY = BlueY';
YellowY = YellowY';

figure(2), scatter3(RedY,GreenY,BlueY,5,'y.')
xlabel('Red')
ylabel('Green')
zlabel('Blue')

YellowBuoySamples = [RedY, GreenY, BlueY];

%% GREEN BUOY

RedG = [];
GreenG = [];
BlueG = [];
YellowG = [];
for k=1:22
   full_name= fullfile(imagepath, fileGreen(k).name);
   I = imread(full_name);
   red = I(:, :, 1);
   green = I(:, :, 2);
   blue = I(:, :, 3);
   yellow = (double(red) + double(green))/2;
   [a b] = size(red);
   for i = 1:a
        for j = 1:b
                RedG(end+1) = red(i,j);
                GreenG(end+1) = green(i,j);
                BlueG(end+1) = blue(i,j);
                YellowG(end+1) = yellow(i,j);
        end
   end
end
RedG = RedG';
GreenG = GreenG';
BlueG = BlueG';
YellowG = YellowG';

figure(3), scatter3(RedG,GreenG,BlueG,5,'g.')
xlabel('Red')
ylabel('Green')
zlabel('Blue')

GreenBuoySamples = [RedG, GreenG, BlueG];

end