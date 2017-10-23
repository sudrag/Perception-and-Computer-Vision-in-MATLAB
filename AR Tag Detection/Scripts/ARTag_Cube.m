clc
close all
clear all

%SOURCE: To detect corners the Douglas-Peucker Algorithm has been used
% Line Simplification by Wolfgang Schwanghart: https://www.mathworks.com/matlabcentral/fileexchange/21132-line-simplification/content/dpsimplify.m

vid = VideoReader('multipleTags.mp4');
nFrames = vid.NumberOfFrames();
num=1;
v = VideoWriter('virtual','MPEG-4');
open(v);

ref_marker = imread('ref_marker.png');
lena = imread('Lena.png');
lena_size = size(lena);

%Camera calibration matrix
fx = 1406.08415449821;
fy = 1417.99930662800;
cx = 1014.13643417416;
cy = 566.347754321696;
s = 2.20679787308599;
% fx = 629.302552;
% fy = 635.529018;
% cx = 960;
% cy = 1000;
% s = 0;
alf = 1;

% K =[1406.08415449821,0,0;2.20679787308599, 1417.99930662800,0;1014.13643417416,566.347754321696,1]';
K = [fx s cx;0 fy cy;0 0 1];

for f = 245 : 365
% f = 120;
f
% if f == 133
%     continue
% end
img1 = read(vid,f);
img = rgb2gray(img1);   
img_size = size(img);
img2 = read(vid,f); %Only for LENA

%% Denoise the image
% Apply a Gaussian filter
H = imgaussfilt(img, 2);

% figure(1)
% subplot(1,2,1), imshow(img);
% title('Original Image')
% subplot(1,2,2), imshow(img_filtered);
% title('Filtered Image')

%% Quad detection

%Edge detection with Roberts method
edges = edge(H,'Roberts');
se1 = strel('disk', 5);
se2 = strel('disk', 4);
edges = imdilate(edges,se1);
edges = imerode(edges,se2);
% figure(2), imshow(edges)

figure('Visible','off')
figure(1), imshow(img1)
hold on

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
% ID of the marker. Look for the binary coding in the 2x2 
               %First plot the corners in the correct orientation
               % Then, get the binary values for it and translate to
               % decimal to get the ID
               b = [0 0 0 0];

               if (pose1 == 1)
                   figure(1), plot(final_pts(1,1),final_pts(1,2),'r.','markersize',20)
                   figure(1), plot(final_pts(2,1),final_pts(2,2),'g.','markersize',20)
                   figure(1), plot(final_pts(3,1),final_pts(3,2),'b.','markersize',20)
                   figure(1), plot(final_pts(4,1),final_pts(4,2),'y.','markersize',20)
                   
                   b(1)=check(4,4);
                   b(2)=check(4,5);
                   b(3)=check(5,5);
                   b(4)=check(5,4);
                   id=binaryVectorToDecimal(b,'LSBFirst');
                   text_str = ['ID: ' num2str(id)];
                   t = text(final_pts(1,1)+150,final_pts(1,2),text_str,'Color','b','FontSize',12);
                   
               elseif (pose2 == 1)
                   figure(1), plot(final_pts(2,1),final_pts(2,2),'r.','markersize',20)
                   figure(1), plot(final_pts(3,1),final_pts(3,2),'g.','markersize',20)
                   figure(1), plot(final_pts(4,1),final_pts(4,2),'b.','markersize',20)
                   figure(1), plot(final_pts(1,1),final_pts(1,2),'y.','markersize',20)
                   
                   b(1)=check(4,5);
                   b(2)=check(5,5);
                   b(3)=check(5,4);
                   b(4)=check(4,4);
                   id=binaryVectorToDecimal(b,'LSBFirst');
                   text_str = ['ID: ' num2str(id)];
                   t = text(final_pts(1,1)+150,final_pts(1,2),text_str, 'Color','b','FontSize',12);
             
               elseif (pose3 == 1)
                   figure(1), plot(final_pts(3,1),final_pts(3,2),'r.','markersize',20)
                   figure(1), plot(final_pts(4,1),final_pts(4,2),'g.','markersize',20)
                   figure(1), plot(final_pts(1,1),final_pts(1,2),'b.','markersize',20)
                   figure(1), plot(final_pts(2,1),final_pts(2,2),'y.','markersize',20)
                   
                   b(1)=check(5,5);
                   b(2)=check(5,4);
                   b(3)=check(4,4);
                   b(4)=check(4,5);
                   id=binaryVectorToDecimal(b,'LSBFirst');
                   text_str = ['ID: ' num2str(id)];
                   t = text(final_pts(1,1)+150,final_pts(1,2),text_str,'Color','b','FontSize',12);

               elseif (pose4 == 1)
                   figure(1), plot(final_pts(4,1),final_pts(4,2),'r.','markersize',20)
                   figure(1), plot(final_pts(1,1),final_pts(1,2),'g.','markersize',20)
                   figure(1), plot(final_pts(2,1),final_pts(2,2),'b.','markersize',20)
                   figure(1), plot(final_pts(3,1),final_pts(3,2),'y.','markersize',20)
                   
                   b(1)=check(5,4); 
                   b(2)=check(4,4);
                   b(3)=check(4,5);
                   b(4)=check(5,5);
                   id=binaryVectorToDecimal(b,'LSBFirst');
                   text_str = ['ID: ' num2str(id)];
                   t = text(final_pts(1,1)+150,final_pts(1,2),text_str,'Color','b','FontSize',12);
               end
               %id
              
%% PROJECTION MATRIX AND CUBE
            if (pose1 == 1 || pose2 == 1 || pose3 == 1 || pose4 == 1)
            % Estimate homography with Unit Square and corner points (lena points)
            m_size=1;
            quad_pts1(:,1) = [0; 0; 1];
            quad_pts1(:,2) = [m_size; 0; 1];
            quad_pts1(:,3) = [m_size; m_size; 1];
            quad_pts1(:,4) = [0; m_size; 1];
            lena_pts1 = lena_pts';
            row_size = size(lena_pts1, 2);
            ones_row = ones(1, row_size);
            lena_pts2 = [lena_pts1; ones_row];
            Hom2 = homography2d(quad_pts1, lena_pts2);
            H2 = Hom2/Hom2(3,3);
            
            %Build the Projection Matrix with the camera intrinsic
            %parameters matrix and the homography matrix
            RT = inv(K)*H2;
            Rt(:,1) = RT(:,1);
            Rt(:,2) = RT(:,2);
            Rt(:,3) = cross(Rt(:,1),Rt(:,2));
            Rt(:,4) = RT(:,3);
            P = K * Rt;

            %Build cube
            %Top of the cube points
            x_c1 = P * [0;0;-1;1];
            x_c1 = x_c1/x_c1(3);
            line_x1=[x_c1(1) lena_pts2(1,1) ];
            line_y1=[x_c1(2) lena_pts2(2,1) ];
            figure(1), line(line_x1,line_y1,'Color','r','LineWidth',1)

            x_c2 = P * [m_size;0;-1;1];
            x_c2 = x_c2/x_c2(3);
            line_x2=[x_c2(1) lena_pts2(1,2) ];
            line_y2=[x_c2(2) lena_pts2(2,2) ];
            figure(1), line(line_x2,line_y2,'Color','r','LineWidth',1)

            x_c3 = P * [m_size;m_size;-1;1];
            x_c3 = x_c3/x_c3(3);
            line_x3=[x_c3(1) lena_pts2(1,3) ];
            line_y3=[x_c3(2) lena_pts2(2,3) ];
            figure(1), line(line_x3,line_y3,'Color','r','LineWidth',1)

            x_c4 = P * [0;m_size;-1;1];
            x_c4 = x_c4/x_c4(3);
            line_x4=[x_c4(1) lena_pts2(1,4) ];
            line_y4=[x_c4(2) lena_pts2(2,4) ];
            figure(1), plot(line_x4,line_y4,'Color','r','LineWidth',1)
            
            line_x5=[x_c1(1) x_c4(1) ];
            line_y5=[x_c1(2) x_c4(2) ];
            figure(1), plot(line_x5,line_y5,'Color','r','LineWidth',1)
            
            line_x6=[x_c1(1) x_c2(1) ];
            line_y6=[x_c1(2) x_c2(2) ];
            figure(1), plot(line_x6,line_y6,'Color','r','LineWidth',1)
            
            line_x7=[x_c2(1) x_c3(1) ];
            line_y7=[x_c2(2) x_c3(2) ];
            figure(1), plot(line_x7,line_y7,'Color','r','LineWidth',1)
            
            line_x8=[x_c3(1) x_c4(1) ];
            line_y8=[x_c3(2) x_c4(2) ];
            figure(1), plot(line_x8,line_y8,'Color','r','LineWidth',1)
%             figure(1), plot(x_c1(1),x_c1(2),'m.','markersize',20)
%             figure(1), plot(x_c2(1),x_c2(2),'c.','markersize',20)
%             figure(1), plot(x_c3(1),x_c3(2),'g.','markersize',20)
%             figure(1), plot(x_c4(1),x_c4(2),'r.','markersize',20)
            end
          
            


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





