%{
% Title:     Small bounding-box filter for small target detection.
% Designer:  Jingneng Fu
% Abstract:  To detect small targets under the condition of dense clutters, 
             we propose a single-frame target detection algorithm based on a small bounding-box filter, 
             which is characterized by good adaptability to the position and size of a small target. 
             During the small target detection process, the proposed algorithm first searches for 
             the local maximum gray pixel and then, a set of concentric bounding boxes whose center
             is the pixel found in the first step is constructed, and the detection thresholds of 
             a neighboring region of this pixel are calculated based on the bounding boxes. 
             Finally, the minimum threshold is used to detect small target pixels in the neighboring region.
             A fast version of the proposed algorithm is a minimum bounding-box filter, 
             which can be implemented by dividing an image into blocks and using the mid-range and range 
             to assess the concentration trend and dispersion of the background. 
             Simulation and analysis results show that the proposed algorithm 
             can achieve high detection probability and low false alarm rate 
             when detecting small targets in the complex background; 
             while its fast version has high computational efficiency. 
             The proposed algorithm can be used in star tracker (refer to demo), 
             infrared searching and tracking systems (refer to reference).
% Keywords:  small target detection, small bounding-box filter,adaptability to position and size, range, mid-range. 
% Demo:      Filtering an image of a star tracker in stray-light backgroud.
% Reference: J. Fu et al.,  Small bounding-box filter for small target detection, Opt.Eng. 60(3), 033107(2021).
% Date:      2021-03-20
%}
clear all;close all;
% Read image 
% ImgIn should be a matrix of double type
fid = fopen('ImgStars.raw','r');
fseek(fid, 10, 'cof');
ImgIn = double(reshape(fread(fid,'uint16'),1024,1024)');
status = fclose(fid);
% Naive small bounding-box filter (NSBBF)
k_NSBBF = 5.0;
tic
ImgOut_NSBBF = NaiSmallBoundBoxFilter(ImgIn,k_NSBBF);
% Computational time(s)
Time_NSBBF = toc
% Fast minimum bounding-box filter (FMBBF)
k_FMBBF = 1.25;
tic
ImgOut_FMBBF = FastMinBoundBoxFilter(ImgIn,k_FMBBF);
% Computational time(s)
Time_FMBBF = toc
%==========================================================================
% Result
% Show image
figure(1);
clims = [130 200];  
imagesc(ImgIn,clims);hold off;
figure(gcf);%colormap(gray);
title('ImgIn');
axis equal; axis off;
figure(2);
clims = [0 50];  
imagesc(ImgOut_NSBBF,clims);hold off;
figure(gcf);%colormap(gray);
title('ImgOut - NSBBF');
axis equal; axis off;
figure(3);
clims = [0 50];  
imagesc(ImgOut_FMBBF,clims);hold off;
figure(gcf);%colormap(gray);
title('ImgOut - FMBBF');
axis equal; axis off;