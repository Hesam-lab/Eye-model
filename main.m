clc;clear; close all;
% In this code, an ocular model is simulated to control gaze in three
% dimensions.
%
% Reference paper: A. D. Polpitaya and B. K. Ghosh, "Modeling the dynamics
% of oculomotor system in three dimensions," 42nd IEEE International
% Conference on Decision and Control (IEEE Cat. No.03CH37475), 2003,
% pp. 6418-6422 Vol.6, doi: 10.1109/CDC.2003.1272353.
%
% Written by: Hesam Shokouh Alaei
%
X1 = [1;1.3;2];   %initial gaze vector
X2 = [1;3;0.7];   % final gaze vector
%% Q Block 
num = X1(1)*X2(1)+X1(2)*X2(2)+X1(3)*X2(3);
det = sqrt(X1(1)^2+X1(2)^2+X1(3)^2)*sqrt(X2(1)^2+X2(2)^2+X2(3)^2);
theta = acos((num)/(det)); % radians
Rx = [1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];
Ry = [cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)];
Rz = [cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1];
R = [cos(theta),sin(theta)^2,sin(theta)*cos(theta);...
    -sin(theta)^2,cos(theta)+(1-cos(theta))*(cos(theta)^2),cos(theta)*sin(theta)*(1-cos(theta));...
    -sin(theta)*cos(theta),cos(theta)*sin(theta)*(1-cos(theta)),cos(theta)+(1-cos(theta))*(sin(theta)^2)];
r1 = 0; %rotation vector in x direction
r2 = 1./(2+2*cos(theta)).*[0;2*sin(theta);0];   %rotation vector in y direction
r3 = 1./(2+2*cos(theta)).*[0;0;2*sin(theta)];   %rotation vector in z direction

%% Activation Generator 
m = 10000;  %constant coefficient 
a1 = 0;
a2 = m.*r2(2);
a3 = m.*r3(3);
A = abs([a1;a2;a3]);

%% Torque Generator
r = 12/1000;    %eye ball radius, m
t = theta*180/pi;   %degree
B = t*0.00006/(r^2);  % viscosity coef, gm/degree
K = t*0.0001/(r^2); %elastic coef
J = t*4.3e-5/(r^2); %moment of inertia of eye ball
lm = 40/1000; % muscle length, m
lo = 60/1000;   % muscle length at rest, m
vm = [5/1000;5/1000;5/1000];  % muscle velocity, m/s
vo = 7/1000;    % muscle velocity at max force, m.s
T = [0;0;0];
we = [-0.3;-0.3;-0.3];
W = [0,we(3),-we(2);-we(3),0,we(1);we(2),-we(1),0];
wh = we;
vm_max = 1/2.5;
for iter=1:100
    temp(:,iter) = we;
    we = we + (T - B.*we - K.*trapz(temp,2))/J;
    vel(:,iter) = we;
    W = [0,we(3),-we(2);-we(3),0,we(1);we(2),-we(1),0];
    R = R + W.*R;
    vm = we.*r;
    Xd(:,iter) = R*X2;  %Desired gaze 
    for i = 1:3
        a = A(i);
        Fl = exp( -0.5*(((lm/lo)-1.05)/0.19)^2);
        Fv = 0.1433/ ( 0.1074+exp(-1.409*sinh(3.2*(vm(i)/vm_max)+1.6 )));
        Fact = Fl*Fv*a;
        Ft = Fact+B.*lm;
        T(i) = r*Ft;   %torque
    end
end
%% plot
figure(1)
plot(vel(1,:))
grid on
title('Angular velocity changes','FontSize',10)
xlabel('iteration','FontSize',10)
ylabel('\omega1 (rad/ms)','FontSize',10)
% axis([0 100 -0.02 0.02])
figure(2)
plot(vel(2,:))
grid on
title('Angular velocity changes','FontSize',10)
xlabel('iteration','FontSize',10)
ylabel('\omega2 (rad/ms)','FontSize',10)
figure(3)
plot(vel(3,:))
grid on
title('Angular velocity changes','FontSize',10)
xlabel('iteration','FontSize',10)
ylabel('\omega3 (rad/ms)','FontSize',10)
figure(4)
starts = zeros(3,iter);
quiver3(starts(1,:), starts(2,:), starts(3,:), Xd(1,:), Xd(2,:), Xd(3,:))
axis equal
title('Movement of gaze vector','FontSize',10)
xlabel('x','FontSize',10)
ylabel('y','FontSize',10)
zlabel('z','FontSize',10)


