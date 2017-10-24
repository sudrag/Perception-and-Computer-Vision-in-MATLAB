function [prob_image] = detectBuoys(I,mean,sigma)
thresh = 0.000005;
%The threshold values of the GMM PDF were found using trial and error
%% Finding the binary thresholds for the buoys
[m n c] = size(I);

data = reshape(I, [m*n, 3]);
dim = 3;
[len, ~] = size(data);
cons = zeros(1,dim);
for i = 1:dim
    cons(i) = 1/((2 * pi) ^ (dim / 2) * det(sigma(:,:,i)) ^ 0.5);
end

prob = zeros(len, dim); % initialize the probability

for i = 1:dim
    difference = double(data) - repmat(mean(i,:),size(data,1), 1);
    part = difference * inv(sigma(:,:,i));
    for j = 1:len
        prob(j,i) = exp(-0.5 * part(j,:) * difference(j,:)') * cons(i);
    end
end

prob_sum = sum(prob, 2);
prob_bi(find(prob_sum > thresh)) = 1;
prob_bi(find(prob_sum <= thresh)) = 0;
prob_image = reshape(prob_bi, m, n);
% imshow(prob_image);



end
