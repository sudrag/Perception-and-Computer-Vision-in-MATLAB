clc
close all
clear all
%SOURCE: To detect corners the Douglas-Peucker Algorithm has been used
% Line Simplification by Wolfgang Schwanghart: https://www.mathworks.com/matlabcentral/fileexchange/21132-line-simplification/content/dpsimplify.m


vid = VideoReader('Tag2.mp4');
nFrames = vid.NumberOfFrames();
num=1;
v = VideoWriter('homography_lena','MPEG-4');
open(v);

ref_marker = imread('ref_marker.png');
lena = imread('Lena.png');
lena_size = size(lena);

for f = 129 : 249
% f = 120;
f
img1 = read(vid,f);
img = rgb2gray(img1);   
img_size = size(img);
img2 = read(vid,f); %Only for LENA

%% Denoise the image
% Apply a Gaussian filter
H = imgaussfilt(img, 2);

%% Quad detection

%Edge detection with Roberts method
edges = edge(H,'Roberts');
se1 = strel('disk', 5);
se2 = strel('disk', 4);
edges = imdilate(edges,se1);
edges = imerode(edges,se2);
% figure(2), imshow(edges)

% Main loop for each of the boundaries in the image
B = bwboundaries(edges, 8, 'noholes'); % Boundary detection
B_size = size(B);
for i=1:B_size(1) %Parent boundaries
    BB = B(i);
    BB_size = size(BB);
    for j=1:BB_size(1) % Inner boundaries
        BB = BB{j};
        ps = dpsimplify(BB,10); %Douglas-Peucker Algorithm
        
        ps_size = size(ps);
        if(ps_size(1) == 5) %If it is a polygon with 4 corners, then detect as quad
            for k=1:ps_size(1)-1
                for kk=1:2 
                    final_corners(k,kk) = ps(k,kk); %Only four corners
                end
            end
            
            % Area filter for small noise
            maxi=max(final_corners,[],1);
            mini=min(final_corners,[],1);
            p=norm(maxi(1)-mini(1));
            q=norm(maxi(2)-mini(2));
            if((p+q)<25)
                break;
            end
     
%             figure(1), plot(final_corners(:,2),final_corners(:,1),'ro')
            
%% HOMOGRAPHY
             %Reference marker points
             quad_size = size(ref_marker);
             quad_pts(1,:) = [1, 1];
             quad_pts(2,:) = [200, 1];
             quad_pts(3,:) = [200, 200];
             quad_pts(4,:) = [1, 200];
             
             %Corner points
             final_pts = [final_corners(:,2), final_corners(:,1)];
             
             %Estimate homography with the 2 sets of four points
             H = fitgeotrans(quad_pts, final_pts,'projective');
             invH = inv(H.T');
             H_1 = projective2d(invH');
              
             %Warp the marker to image plane
             RA = imref2d([quad_pts(3,1) quad_pts(3,2)], [1 quad_pts(3,1)-1], [1 quad_pts(3,1)-1]);
             [warp,r] = imwarp(img, H_1, 'OutputView', RA);              
%              figure, imshow(warp);
             
             th = graythresh(warp);
             markBin = im2bw(warp, th);
%              se3 = strel('square', 1);
%              markBin = imerode(markBin,se3);
%              figure, imshow(markBin)
            
             
%% GRID CREATION AND BITS DETECTION

            %Create the 8x8 grid
            maxi=max(quad_pts,[],1);
            mini=min(quad_pts,[],1);
            p=norm(maxi(1)-mini(1));
            q=norm(maxi(2)-mini(2));
            for iii=1:8
              m(iii)=round(mini(1)+(iii-1)/8*p);
              n(iii)=round(mini(2)+(iii-1)/8*q);
            end
            Agrid=markBin;
           for iii=1:8
               for jjj=mini(2):maxi(2)
                   Agrid((m(iii)),jjj) = 1; 
               end
           end  
           for jjj=1:8
               for iii=mini(1):maxi(1)
                   Agrid(iii,n(jjj)) = 1; 
               end
           end
%            figure 
%            imshow(Agrid);hold on;
           m(9)=maxi(1);
           n(9)=maxi(2);
           
           %Get the bits of each of the cells of the grid
           intensity=0;
           for iii=1:8
            for jjj=1:8  
                   length=0;
                   for kk=(m(iii):m(iii+1))
                      breadth=0;
                      length=length+1;
                      for l=(n(jjj):n(jjj+1))
                      breadth=breadth+1;
                        intensity=intensity+markBin(kk,l);
                      end
                   end
                   temp=intensity;
                   intensity=intensity-temp;
                   if((temp+30)>(length*breadth))%a tolerance of ~ pixels is provided
                     check(iii,jjj)=1;
                   else
                     check(iii,jjj)=0;
                   end
            end
           end
 % Orientation of the marker
           % First condition: marker has to outer rows and columns of 0
           % values 
           if check(1:2,:) == zeros(2,8) 
              if check(:,1:2) == zeros(8,2)
                  if check(7:8,:) == zeros(2,8)
                      if check(:,7:8) == zeros(8,2)
                          if (check(6,6) || check(6,3) || check(3,6) || check(3,3))
                            flag = 1;
                          end
                      end
                  end
              end
           else flag = 0;
           end
           if flag == 1 
               %Look for a 1 in one of the corners of interior 4x4
               pose1 = 0; pose2 = 0; pose3 = 0; pose4 = 0;
               if (check(6,6) == 1)
                   pose1 = 1; pose2 = 0; pose3 = 0; pose4 = 0;
                   lena_pts = [final_pts(1,1),final_pts(1,2);final_pts(2,1),final_pts(2,2);final_pts(3,1),final_pts(3,2);final_pts(4,1),final_pts(4,2)];
               end
               if (check(6,3) == 1)
                   pose1 = 0; pose2 = 1; pose3 = 0; pose4 = 0;
                   lena_pts = [final_pts(2,1),final_pts(2,2);final_pts(3,1),final_pts(3,2);final_pts(4,1),final_pts(4,2);final_pts(1,1),final_pts(1,2)];
               end
               if (check(3,3) == 1)
                   pose1 = 0; pose2 = 0; pose3 = 1; pose4 = 0;
                   lena_pts = [final_pts(3,1),final_pts(3,2);final_pts(4,1),final_pts(4,2);final_pts(1,1),final_pts(1,2);final_pts(2,1),final_pts(2,2)];
               end
               if (check(3,6) == 1)
                   pose1 = 0; pose2 = 0; pose3 = 0; pose4 = 1;
                   lena_pts = [final_pts(4,1),final_pts(4,2);final_pts(1,1),final_pts(1,2);final_pts(2,1),final_pts(2,2);final_pts(3,1),final_pts(3,2)];
               end
              
%% LENA
               % Estimate homography with the corners of the Lena image
               % (source points) and the corners with a correct
               % orientation (lena_pts)
               src_pts = [1, 1;lena_size(1), 1;lena_size(1), lena_size(2);1, lena_size(1)];
               H_lena = fitgeotrans( src_pts, lena_pts,'projective');
               
               %Warp image and substitute the pixel values on the original
               %with the Lena pixel values 
               RA2 = imref2d([img_size(1) img_size(2)], [1 img_size(2)-1], [1 img_size(1)-1]);
               [warp_lena,r2] = imwarp(lena, H_lena, 'OutputView', RA2);
               warplena_size = size(warp_lena);
%                figure, imshow(warp_lena)
               for i_l=1:img_size(1)
                   for j_l=1:img_size(2)
                       for k_l=1:3
                           if(warp_lena(i_l,j_l,k_l)~=0)
                               img2(i_l,j_l,k_l) = warp_lena(i_l,j_l,k_l);
                           end
                       end
                   end
               end
               figure(2), imshow(img2)
               hold on

            else %break if not a tag
                break;
           end
       
        end
    end
end

frame = getframe(gca);
writeVideo(v,frame);

num = num + 1;
pause(0.001)
end

%% Create video
close(v);