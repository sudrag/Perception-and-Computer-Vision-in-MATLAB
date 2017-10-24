clear all
clc
%


%% CLASSIFIER
training_folder = fullfile('P2_Submission','TSR','input','training_');
trainingSet = imageSet(training_folder,   'recursive');
% testing_folder = fullfile('testing_');
% testingSet = imageSet(testing_folder, 'recursive');

trainingFeatures = [];
trainingLabels   = [];

for i = 1:numel(trainingSet)
    
   numImages = trainingSet(i).Count;
   hog = [];
   
   for j = 1:numImages
       img = read(trainingSet(i), j);
       
       %Resize Image to 64x64
       img = im2single(imresize(img,[64 64]));
       %Get HOG Features
       hog_cl = vl_hog(img, 4);
       [hog_1, hog_2] = size(hog_cl);
       dim = hog_1*hog_2;
       hog_cl_trans = permute(hog_cl, [2 1 3]);
       hog_cl=reshape(hog_cl_trans,[1 dim]); 
       hog(j,:) = hog_cl;
   end
   labels = repmat(trainingSet(i).Description, numImages, 1);
   
   trainingFeatures = [trainingFeatures; hog];
   trainingLabels = [trainingLabels; labels];
end

%SVM
classifier = fitcecoc(trainingFeatures, trainingLabels);



%% MAIN ALGORITHM  

cd 'P2_Submission'
cd 'TSR'
cd 'input'
cd 'input'
files = ls('*jpg');
names = files;

v = VideoWriter('Sign_detection','MPEG-4');
v.FrameRate = 30;
open(v);

for fr = 90:2650%size(names,1)
    fr
    name = names(fr,:);
    img = imread(name);
%     figure(1), imshow(img)
    
    %Gaussian Filter
    mu = 5;
    sigma = 2;
    index = -floor(mu/2) : floor(mu/2);
    [X Y] = meshgrid(index, index);
    H = exp(-(X.^2 + Y.^2) / (2*sigma*sigma));
    H = H / sum(H(:));
    I = imfilter(img, H);
    
    % RGB Normalization
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);

    red = uint8(max(0, min(R-B, R-G)));
    blue = uint8(max(0, min(B-R, B-G)));

    bb = im2bw(blue,.15);
    br = im2bw(red,.15);

    rb = red + blue;

    %Apply mask to the image
    x = [1 1628 1628 1];
    y = [1 1 618 618];
    mask = poly2mask(x,y, 1236, 1628);
    crop = uint8(immultiply(rb,mask));
%     figure(2), imshow(crop)
    
    % MSER
    [r,f] = vl_mser(crop,'MinDiversity',0.7,...
                    'MaxVariation',0.2,...
                    'Delta',8);

    M = zeros(size(crop));
    for x=r'
     s = vl_erfill(crop,x);
     M(s) = M(s) + 1;
    end
    
    thresh = graythresh(M);
    M = im2bw(M, thresh);
    se = strel('octagon',6);
    M = imdilate(M, se);
%     figure(2), imshow(M)

    % Area filter
    M = bwareafilt(M, [950 10000]);
%     figure(2), imshow(M)

    regions = regionprops( M, 'BoundingBox');

    %Get Bounding boxes for the blobs given by MSER
    figure(3) ;
    clf; imagesc(img) ; hold on ; axis equal off; colormap gray ;    
    for k = 1 : length(regions)
      box = regions(k).BoundingBox;
      ratio = box(3)/box(4);
      if ratio < 1.1 && ratio > 0.6 %Aspect Ration of detections
%           rectangle('Position', box,'EdgeColor','r','LineWidth',2 )
          sign = imcrop(img, box);  
          sign = im2single(imresize(sign,[64 64]));
          %figure, imshow(sign)

          % CLASSIFICATION
          %Get HOG Features of detections 
          hog = [];
          hog_det = vl_hog(sign, 4);
          [hog_1, hog_2] = size(hog_det);
          dim = hog_1 * hog_2;
          hog_det_trans = permute(hog_det, [2 1 3]);
          hog_det=reshape(hog_det_trans,[1 dim]); 
          hog = hog_det;
          [predictedLabels, score] = predict(classifier, hog);
%           label = str2num(predictedLabels);
          for j = 1:length(score)
              if (score(j) > -0.04)
                rectangle('Position', box,'EdgeColor','g','LineWidth',2 )
%                 label
%                 score
                cd ..
                cd 'Sample'
                samname=strcat(predictedLabels,'.png');
                im=imread(samname);
                im = im2single(imresize(im,[box(4) box(3)]));
                image([int64(box(1)-box(3)) int64(box(1)-box(3)) ],[int64(box(2)) int64(box(2))],im);
              end
          end
      end    
    end
    
    frame = getframe(gca);
    writeVideo(v,frame);

    pause(0.001);
%     cla
end
cd ..
cd ..
cd ..
cd ..

close(v)
