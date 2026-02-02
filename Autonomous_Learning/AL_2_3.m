clearvars
close

% Training set for GP: x input, y output
x=[-0.8372, -0.4558, 0.6902, 0.1114, -0.4678];
y=[-0.6414, -1.0286, -0.6893, -1.4021, -1.0594];

% Test points x*
x_star = [-0.5,0.5];

GP_Regression(x,y,x_star,1)

xs = linspace(-1,1,100);
L = [0.1,0.4,0.7,1.0,1.3,1.6]
grid on;

for i=1:6
    subplot(2,3,i);
    hold on;
    scatter(x,y,'g', 'filled')
    [y_star, y_star_lower, y_star_upper] = GP_Regression(x, y, x_star, L(i));
    scatter(x_star, y_star, 'r', 'filled');
    [ys, ys_lower, ys_upper] = GP_Regression(x,y,xs,L(i));
    plot(xs,ys,'b')
    plot(xs,ys_lower, 'r', LineStyle='-.')
    plot(xs,ys_upper, 'r', LineStyle='-.')
    title(['l = ', num2str(L(i))]);
end

