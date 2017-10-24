%% Creating an XML 
clc
 clear all
 data=[]
 neg_data=[]
 boxes=[]
% cars_directory = fullfile('Project 3', 'Dataset', 'vehicles');
% non_cars_directory = fullfile('Project 3', 'Dataset', 'non-vehicles');
% PositiveSet = imageSet(cars_directory, 'recursive');
% %PositiveSet = imageSet(cars_directory);
% NegativeSet = imageSet(non_cars_directory ,'recursive');
% % for i=1:1
%    names=PositiveSet(1,1).ImageLocation';
%    data=[data;names];
%    names=PositiveSet(1,3).ImageLocation';
%    data=[data;names];
% % end
% for i=1:2
%     names=NegativeSet(1,i).ImageLocation';
%     neg_data=[neg_data;names];
% end
% box=[1,1,64,64];
% pos_data=struct('imageFilename',data,'objectBoundingBoxes',box);
% trainCascadeObjectDetector('Car3.xml',pos_data,neg_data,'NegativeSamplesFactor',1,'FalseAlarmRate',0.02,'NumCascadeStages',4);
%% INITIAL DETECTION
cd('Project 3/Dataset')
vid = VideoReader('simple.avi');
cd ..
cd ..
% v = VideoWriter('12345','MPEG-4');
% v.FrameRate = 30;
% open(v);
u=8;
frame = read(vid,1);
detector = vision.CascadeObjectDetector('Car2A.xml');
img = frame;
%img = imgaussfilt(img,1.5);
img_mask=img;
 x = [230 630 630 230];
 y = [310 310 470 470];
% x = [1 630 630 1];
% y = [1 1 470 470];
 poly_top = y(2) + 20;
 mask = poly2mask(x,y, 480, 704);
img_mask(:,:,1) = immultiply(img(:,:,1),mask);
img_mask(:,:,2) = immultiply(img(:,:,2),mask);
img_mask(:,:,3) = immultiply(img(:,:,3),mask);
bbox1 = step(detector,img_mask);
detectedImg2 = insertObjectAnnotation(img,'rectangle',bbox1,'Car');
r=size(bbox1,1);
for i1=1:r
    if (i1>r)
            break
    end
    area=bbox1(i1,3)*bbox1(i1,4);
    if (area>10000 || area<1200)
        bbox1(i1,:)=[];
        r=r-1;            
    end
end
   track = [];
   bbox1;
        sizeB = size(bbox1);
        rowsgB = size(1);
        for a = 1:sizeB
            for b = 1:sizeB
                B1 = bbox1(b,:);
                B2 = bbox1(a,:);
                q=0;
                if (B2(1)>B1(1) && B2(1)+B2(3)<B1(1)+B1(3))% if x wihtin x and y within y
                    track = [track; b];
                end
            end
        end
        bbox1(track,:) = [];
%% DETECTING AT STEPS OF SIZE U
for f=1:u:290
frame = read(vid,f);
f
detector = vision.CascadeObjectDetector('Car2A.xml');
img = frame;
img_mask=img;
%ROI
 x = [230 630 630 230];
 y = [310 310 470 470];
 poly_top = y(2) + 20;
 mask = poly2mask(x,y, 480, 704);
 %Creating mask
img_mask(:,:,1) = immultiply(img(:,:,1),mask);
img_mask(:,:,2) = immultiply(img(:,:,2),mask);
img_mask(:,:,3) = immultiply(img(:,:,3),mask);
bbox = step(detector,img_mask);
detectedImg2 = insertObjectAnnotation(img,'rectangle',bbox,'Car');
r=size(bbox,1);
for i1=1:r
    if (i1>r)
            break
    end
    area=bbox(i1,3)*bbox(i1,4);
    %area filter
    if (area>10000 || area<1600)
        bbox(i1,:)=[];
        r=r-1;            
    end
end
   track = [];
   bbox;
        sizeB = size(bbox);
        rowsgB = size(1);
        for a = 1:sizeB
            for b = 1:sizeB
                B1 = bbox(b,:);
                B2 = bbox(a,:);
                q=0;
                if (B2(1)>B1(1) && B2(1)+B2(3)<B1(1)+B1(3))% if x wihtin x and y within y
                    track = [track; b];
                end
            end
        end
        bbox(track,:) = [];

detectedImg=img;
detectedImg = insertObjectAnnotation(img,'rectangle',bbox,'Car');
figure(1);
imshow(detectedImg);
%% KLT Tracking
frame=read(vid,f);
%Track till next detection
for l=f:f+u-1
frame1=read(vid,l);
posi=zeros(4,4);
queue=[0,0,0,0,0];
figure(2), imshow(frame1), hold on, title('Detected features');    
for r1=1:size(bbox)
bboxPoints = bbox2points(bbox(r1, :));
points = detectMinEigenFeatures(rgb2gray(frame), 'ROI', bbox(r1,:),'MinQuality',0.1);
%Using point tracker from CV toolbox
pointTracker = vision.PointTracker('MaxBidirectionalError', 4);
points = points.Location;
initialize(pointTracker, points, frame);
oldPoints = points;
    frame3 = read(vid,l+1);
    [points, isFound] = step(pointTracker, frame3);
    visiblePoints = points(isFound, :);
    oldInliers = oldPoints(isFound, :);
    
    if size(visiblePoints, 1) >= 2 % Increasing this can reduce smaller featured false detections
        %Finding transform between tracked features
        [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
            oldInliers, visiblePoints, 'similarity', 'MaxDistance',1);
        
        % Apply the transformation to the bounding box points
        bboxPoints = transformPointsForward(xform, bboxPoints);
        % Plotting the bounding box
        bboxPolygon = reshape(bboxPoints', 1, []);
        xPoints = [bboxPolygon(1) bboxPolygon(3) bboxPolygon(5) bboxPolygon(7)];
        yPoints = [bboxPolygon(2) bboxPolygon(4) bboxPolygon(6) bboxPolygon(8)];
        xmin = min(xPoints);
        ymin = min(yPoints);
        xmax = max(xPoints);
        ymax = max(yPoints);
        figure(2),
       color=['r','g','b','w','y','w'];
        pos=[xmin ymin xmax-xmin ymax-ymin];
        for i2=1:size(bbox1)
        %Limiting tracking to a certain region around the previous
        %detection
        if(sqrt((pos(1)-bbox1(i2,1))^2+(pos(2)-bbox1(i2,2))^2)<32 && sqrt((pos(1)+pos(3)-bbox1(i2,1)-bbox1(i2,3))^2+(pos(2)-bbox1(i2,2))^2)<32 && queue(i2)==0)
        rectangle('Position',pos,'EdgeColor',color(i2),'linewidth',2); 
        bbox1(i2,:)=pos;
        else
            continue;
        end
        end
        tmp=sum(queue);
        %The tracked features
        frame3 = insertMarker(frame, visiblePoints, '+', ...
            'Color', 'white'); 
        hold on
        oldPoints = visiblePoints;
        setPoints(pointTracker, oldPoints); 
        release(pointTracker)
    end
end
for i3=1:4
   if(queue(i3)==0)
   rectangle('Position',bbox1(i3,:),'EdgeColor',color(i3),'linewidth',2);
   end
end
frame2 = getframe(gca);
writeVideo(v,frame2);
%release(pointTracker);

end
end
close(v);

