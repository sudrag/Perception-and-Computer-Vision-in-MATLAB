clear all
clc
close all

%Extract the camera parameters for each image
[fx, fy, cx, cy, G_camera_image, LUT] = ReadCameraModel('Oxford_dataset/stereo/centre','Oxford_dataset/model');

K = [fx, 0, cx;
     0, fy, cy;
     0, 0, 1];
cameraParams = cameraParameters('IntrinsicMatrix',K');
 
pos = [0 0 0];
Rpos = [1 0 0;
        0 1 0
        0 0 1];

v = VideoWriter('VisualOdom','MPEG-4');
v.FrameRate = 25;
open(v);

cd Oxford_dataset/stereo/centre

images.filename = ls('*png');
size_im = size(images.filename); 

for i = 200:size_im(1)-1
    i;
    
    %From Bayer to RGB
    I = imread(images.filename(i,:));
    J = demosaic(I,'gbrg');
    % imshow(I);
%     figure(1), imshow(J);
    
    %Get the next frame from the current
    I_next = imread(images.filename(i+1,:));
    J_next = demosaic(I_next,'gbrg');
    % imshow(I);
    %figure(2), imshow(J_next);
    
    %Undistort both images (i and i+1)
    img = UndistortImage(J, LUT);
    %figure(3), imshow(img)
    img_next = UndistortImage(J_next, LUT);
    %figure(4), imshow(img_next)
    
    %%Denoise image
    img = imgaussfilt(img, 0.8);
    img_next = imgaussfilt(img_next, 0.8);
%     figure(3), imshow(img)
%     figure(4), imshow(img_next)

    %%Gray Image
    img = rgb2gray(img);
    img_next = rgb2gray(img_next);
    %figure(3), imshow(img)
    %figure(4), imshow(img_next)
    
    %% Feature extraction from both the images (Harris or FAST)
%     harris1 = detectHarrisFeatures(img);
%     harris2 = detectHarrisFeatures(img_next);
    harris1 = detectSURFFeatures(img);
    harris2 = detectSURFFeatures(img_next); 
    
    [features1,valid_points1] = extractFeatures(img, harris1);
    [features2,valid_points2] = extractFeatures(img_next, harris2);
    
    indexPairs = matchFeatures(features1,features2, 'MaxRatio', 0.2);
    matchedPoints1 = valid_points1(indexPairs(:,1),:);
    matchedPoints2 = valid_points2(indexPairs(:,2),:); 
    
%     figure(5); showMatchedFeatures(img, img_next, matchedPoints1, matchedPoints2);
    
    x_good = matchedPoints1.Location(:,1);
    y_good = matchedPoints1.Location(:,2);
    x_good_next = matchedPoints2.Location(:,1);
    y_good_next = matchedPoints2.Location(:,2);
    x11 = [x_good'; y_good'];
    x22 = [x_good_next'; y_good_next'];
    
    [~, inliers] = ransacfitfundmatrix(x11, x22, 0.001);
    for j = 1:size(inliers(2))
       x_good = x_good(inliers); 
       y_good = y_good(inliers);
       x_good_next = x_good_next(inliers);
       y_good_next = y_good_next(inliers);
    end
    
%     
    %% Fundamental Matrix (Done somehow like Kovesi) 8 point-method
    indices = [];
    sizeg = size(x_good);

    x1 = x_good; y1 = y_good;
    x2 = x_good_next; y2 = y_good_next;
    
    X1 = [x1'; y1'; ones(1, sizeg(1))];
    X2 = [x2'; y2'; ones(1, sizeg(1))];
    
    [p1, T1] = normalise2dpts(X1);
    [p2, T2] = normalise2dpts(X2);
    
    %Plug the 8 matches into the Fundamental Matrix
    Y = [p2(1,:)'.*p1(1,:)'   p2(1,:)'.*p1(2,:)'  p2(1,:)' ...
         p2(2,:)'.*p1(1,:)'   p2(2,:)'.*p1(2,:)'  p2(2,:)' ...
         p1(1,:)'             p1(2,:)'            ones(sizeg(1),1) ]; 
    
    [U, S, V] = svd(Y, 0);
    V_1 = reshape(V(:, 9), [3 3])';
    
    [FU, FD, FV] = svd(V_1,0);
    F = FU*diag([FD(1,1) FD(2,2) 0])*FV';
    
    %Fundamental Matrix and Matlab Fund Matrix
    F = T2'*F*T1;
    F = F / norm(F);
    if F(end) < 0
       F = -F; 
    end
    fRANSAC = F;
    inliers1 = [x_good y_good];
    inliers2 = [x_good_next y_good_next];

    %% Fundamental Matrix with RANSAC
%     [fRANSAC, inliersIdx] = estimateFundamentalMatrix(matchedPoints1,matchedPoints2,'Method','norm8Point','NumTrials',2000,'DistanceThreshold',1e-3);
%     m1X = matchedPoints1.Location(:,1);
%     m1Y = matchedPoints1.Location(:,2);
%     inliers1 = [m1X(inliersIdx) m1Y(inliersIdx)];
%     
%     m2X = matchedPoints2.Location(:,1);
%     m2Y = matchedPoints2.Location(:,2);
%     inliers2 = [m2X(inliersIdx) m2Y(inliersIdx)]; 
    %% Essential Matrix
    E = K' * fRANSAC * K;

    [U, D, V] = svd(E);
    e = (D(1,1) + D(2,2)) / 2;
    D(1,1) = 1;
    D(2,2) = 1;
    D(3,3) = 0;
    E = U * D * V';
    [U, ~, V] = svd(E);

    W = [0 -1 0;
         1 0 0; 
         0 0 1];
     
    R1 = U * W * V';
    if det(R1) < 0
        R1 = -R1;
    end
    R2 = U * W' * V';
    if det(R2) < 0
        R2 = -R2;
    end
   
    t1 = U(:,3)';
    t2 = -t1;
    
    Rs = cat(3, R1, R1, R2, R2);
    Ts = cat(1, t1, t2, t1, t2);
    
    %% Choose the right solution for rotation and translation
    numNegatives = zeros(1, 4);
    P1 = cameraMatrix(cameraParams, eye(3), [0,0,0]);
%     P1 = 
    for k = 1:size(Ts, 1)
       P2 = cameraMatrix(cameraParams,Rs(:,:,k)', Ts(k, :));
    % Triangulation
       points3D_1 = zeros(size(inliers1, 1), 3, 'like', inliers1);
       P1_a = P1';
       P2_a = P2';
       
       M1 = P1_a(1:3, 1:3);
       M2 = P2_a(1:3, 1:3);
       
       c1 = -M1 \ P1_a(:,4);
       c2 = -M2 \ P2_a(:,4);

       for kk = 1:size(inliers1,1)
          u1 = [inliers1(kk,:), 1]';
          u2 = [inliers2(kk,:), 1]';
          a1 = M1 \ u1;
          a2 = M2 \ u2;
          A = [a1, -a2];
          y = c2 - c1;
          
          alpha = (A' * A) \ A' * y;
          p = (c1 + alpha(1) * a1 + c2 + alpha(2) * a2) / 2;
          points3D_1(kk, :) = p';
       end
       %Triangulation ends
       points3D_2 = bsxfun(@plus, points3D_1 * Rs(:,:,k)', Ts(k, :));
       numNegatives(k) = sum((points3D_1(:,3) < 0) | (points3D_2(:,3) < 0));
    end
    
    [val, idx] = min(numNegatives);
    R = Rs(:,:,idx)';
    t = Ts(idx, :);
    tNorm = norm(t);
    if tNorm ~= 0
        t = t ./ tNorm;
    end
    
    
    %% Rotation and translation
    R = R';
    t = -t * R;
    
    %% Trajectory
    Rpos = R * Rpos;
    
    pos = pos + t * Rpos;
    
    figure(8)
    subplot(1,2,1)
    title('Camera Feed')
    imshow(J)
    subplot(1,2,2)
    title('Trajectory')
    plot(pos(1),pos(3),'ro')
    hold on
    
    frame = getframe(gcf);
    writeVideo(v,frame);
    
    pause(0.005)
end

close(v)

cd ../../..










