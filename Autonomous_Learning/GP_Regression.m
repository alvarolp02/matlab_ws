function [y, y_lower, y_upper] = GP_Regression(x_train, y_train, x, l)
% x_train: Input training data points (1D array)
% y_train: Corresponding output values for the training data points (1D array)
% x: Test data points where predictions are to be made (1D array)
% l: Length scale parameter for the kernel function
% y: Predicted output values at the test data points (1D array)

% Initialize the covariance matrix K and K_star
K = zeros(length(x_train), length(x_train));
K_star = zeros(length(x_train), length(x));

% Squated exponential kernel for GP regression
function [res] = k(x1,x2)
sigma_f=1;
sigma_n=0.5;
delta = 0;
if x1==x2
    delta=1;
end

res=sigma_f^2 * exp(-(0.5*(l^-2))*((x1-x2).^2)) + delta*sigma_n^2;
end

% Compute the Gramm matrix (covariance of training points using kernel)
for i=1:length(x_train)
    for j=1:length(x_train)
        K(i,j) = k(x_train(i), x_train(j));
    end
end

% Compute the Cholesky decomposition of K
L = chol(K)';

% Compute the vector alpha
alpha = L' \ (L \ y_train');

% Compute K*, the kernel function applied between the test points and all
% training points

for i=1:length(x_train)
    for j=1:length(x)
        K_star(i,j) = k(x_train(i), x(j));
    end
end

% Final prediction: 
y = K_star'*alpha;

% Compute the variance of the predictions
v = L \ K_star;

% Variance of the predictions at the test points
var_y = k(x, x) - v' * v;

% Confidence intervals (95% confidence)
confidence_interval = 1.96 * sqrt(diag(var_y));

% Lower and upper bounds of the confidence intervals
y_lower = y - confidence_interval;
y_upper = y + confidence_interval;

end