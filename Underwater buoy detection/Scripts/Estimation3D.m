function [muR, covR, muG, covG, muY, covY] = Estimation3D()


%Run the color distribution Training function
[RedBuoySamples, YellowBuoySamples, GreenBuoySamples] = Color_Distribution();

%% RED BUOY
muR = [mean(RedBuoySamples(:,1)), mean(RedBuoySamples(:,2)), mean(RedBuoySamples(:,3))];
arrR = [double(RedBuoySamples(:,1)), double(RedBuoySamples(:,2)), double(RedBuoySamples(:,3))];
covR = cov(arrR);

%% GREEN BUOY
muG = [mean(GreenBuoySamples(:,1)), mean(GreenBuoySamples(:,2)), mean(GreenBuoySamples(:,3))];
arrG = [double(GreenBuoySamples(:,1)), double(GreenBuoySamples(:,2)), double(GreenBuoySamples(:,3))];
covG = cov(arrG);

%% YELLOW BUOY

muY = [mean(YellowBuoySamples(:,1)), mean(YellowBuoySamples(:,2)), mean(YellowBuoySamples(:,3))];
arrY = [double(YellowBuoySamples(:,1)), double(YellowBuoySamples(:,2)), double(YellowBuoySamples(:,3))];
covY = cov(arrY);

end