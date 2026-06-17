% MAKE SURE TO HAVE vx, vy, r, x, y, yaw, imu_ax, imu_ay, imu_r, steer, wencs AND torques VECTORS WITH THE SAME SIZE 

%% GGV
v_abs = sqrt(vx.*vx+vy.*vy);
delta = -0.06;
vxR = vx*cos(delta)-vy*sin(delta);
vyR = vx*sin(delta)+vy*cos(delta);
hold on
grid on
axis equal
plot(vy,vx)
plot(vyR,vxR)

%%

delta2 = deg2rad(-3.5);
axR = imu_ax*cos(delta2)-imu_ay*sin(delta2);
ayR = imu_ax*sin(delta2)+imu_ay*cos(delta2);
hold on
scatter(imu_ay,imu_ax)
grid on
scatter(ayR,axR)
% grid on
%%
scatter3(imu_ax,imu_ay,v_abs)

ay_max_vel_coeff = [0.95833, 0.03825, 0.54782];
% ay_max_vel_coeff = [1.6, 0.03825, 0.3];
ax_max_vel_coeff = [1.051146, 1.51092];

g=9.81;
m = 170; % Mass
cl = 5.0;
cd = 1.9;
A = 1;
ro = 1.2;

mux = 1*1.3291;
muy = 1*1.5863;
v = 20;

ax_max = mux*g*ax_max_vel_coeff(1) + 0.5*(ro*A/m)*(mux*cl - cd)*ax_max_vel_coeff(2)*v*v;

ay_max = muy*g* ay_max_vel_coeff(1) + cl*ay_max_vel_coeff(2)*v + 0.5*(ro*A/m)*(muy*cl)*ay_max_vel_coeff(3)*v*v;

hold on

vs = linspace(0,30,60);
t = linspace(0,2*pi,500);

for i = 1:length(vs)

    v = vs(i);

    ax_max = mux*g*ax_max_vel_coeff(1) ...
           + 0.5*(ro*A/m)*(mux*cl - cd)*ax_max_vel_coeff(2)*v^2;

    ay_max = muy*g*ay_max_vel_coeff(1) ...
           + cl*ay_max_vel_coeff(2)*v ...
           + 0.5*(ro*A/m)*(muy*cl)*ay_max_vel_coeff(3)*v^2;

    semi_x = ax_max;
    semi_y = ay_max;

    x = semi_x*cos(t);
    y = semi_y*sin(t);
    z = v*ones(size(t));

    plot3(x,y,z,'b')
end

axis equal
grid on
xlabel('a_x [m/s^2]')
ylabel('a_y [m/s^2]')
zlabel('v [m/s]')
title('GGV envelope')
view(3)