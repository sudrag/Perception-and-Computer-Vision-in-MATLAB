function [ mu, sigma ] = Gmm( data, dim)

% Implement the estimation of the parameters for Gaussian Misture Model.
% Input: train_data: the three-dim data of Yellow Ball Area
%        dim: number of dimensions of GMM
% Output: mu: mean of Gmm
%         sigma: covariance of Gmm

len = size(data, 1);
%% Initialize the mu and sigma 
mu = rand(dim, 3);    % mu is a random value between max and min 
for i = 1:dim
    mu(i,1) = mu(i,1)*(max(data(:,1)) - min(data(:,1))) + min(data(:,1));
    mu(i,2) = mu(i,2)*(max(data(:,2)) - min(data(:,2))) + min(data(:,2));
    mu(i,3) = mu(i,3)*(max(data(:,3)) - min(data(:,3))) + min(data(:,3));
end
% the size of sigma matrix is 3 * 3 * dim
sigma = zeros(3, 3, dim);
for i = 1:dim
    sigma(:,:,i) = [1,0.1,0.2;0.1,0.5,0.4;0.2,0.4,2]; 
end

%% EM algorithm
diff = 100;   
Threshold = 0.00001;

while diff > Threshold
    % update z
    prob  = g_prob( data, mu, sigma, dim ); 
    z_matrix = prob ./ repmat(sum(prob,2), 1, dim);
    % the prob value can be very small, to prevent the NaN in z_dim, we substitute all
    % NaN with 0.25 
    z_matrix(find(isnan(z_matrix))) = 0.33;
    
    z_dim = sum(z_matrix, 1);
    mu_pre = mu;
    sigma_pre = sigma;
    
    % update mu
    for i = 1:dim
        mu(i, :) = 1 / z_dim(i) * (z_matrix(:,i)' * double(data));
    end
    
    % update sigma
    for i = 1:dim
        mu_data_diff = double(data) - repmat(mu(i,:),size(data,1), 1);
        temp = zeros(3,3);
        for j = 1:len
            temp = temp + mu_data_diff(j,:)' * mu_data_diff(j,:) .* z_matrix(j,i);
        end
        sigma(:,:,i) = temp / z_dim(i);      
    end
       
    % compute diff
    diff = sum(sum(sum(abs(sigma - sigma_pre)))) + sum(sum(abs(mu - mu_pre)));
    diff
           
end




