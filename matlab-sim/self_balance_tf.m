% Model of self balancing robot for PID controller
% See scheme anf formulas on notebook

clear all
clc

load('params.mat')


I_wheel = M_wheel*(r^2); %placeholder
I_body = M_body*(l^2)/3; %placeholder

nums1 = -(M_body + 2*(I_wheel/r^2) + 2*M_wheel + M_body*l/r);
nums0 = -2*b/r;
num = [nums1 nums0];

dens3 = I_body * M_body + 2*I_wheel*(I_body*M_body*l^2)/(r^2)  + 2*I_body*M_wheel + 2*M_wheel*M_body*l^2;
dens2 = (2*b/r)*(I_body+M_body*g);
dens1 = -( ((M_body^2)*g*l) + 2*(I_wheel*M_body*g*l)/(r^2) + 2*M_body*M_wheel*g*l );
dens0 = -2*(b*M_body*g*l) / r;
den = [dens3 dens2 dens1 dens0];

w = tf(num, den)

sys = zpk(w);
sys.DisplayFormat='roots'

%sys.Z{1}
% sys.P{1}(1)

K = sys.K
p1 = sys.P{1}(1)
p2 = sys.P{1}(3)
p3 = sys.P{1}(2)

%display('Kd must be [' + num2str(p3/K) + ', 0]');

% Scelgo Kd così
kd = -0.001
kponkd = (p1+p2);
kionkd = (p1*p2);
kp = kponkd*kd
ki = kionkd*kd

% rlocus(sys, -sys)
%{
bode(w)
figure;
nyquist(w)
%}
