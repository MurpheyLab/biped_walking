function [  ] = animate2( t, x )
%Homogeneous transformation
R = @(angle) [cos(angle), -sin(angle);
            sin(angle), cos(angle)];
G = @(R, x) [R, x;
                0, 0, 1];
%Params
l = 0.5; r = 1; MH = 15; m = 5; g=9.81; MT = 10;

%Kinematics
gh_f = G(R(0), [x(4);x(5)]);%Hip wrt global frame
g_T_f = gh_f*G(R(-x(3)), [0;0])*G(R(0), [0;l]);%Torso

g_f1_f = gh_f*G(R(-x(1)), [0;0])*G(R(0), [0;-r/2]);%femur 1
g_f2_f = gh_f*G(R(x(2)), [0;0])*G(R(0), [0;-r/2]);%femur 2
gtoe1_f = g_f1_f*G(R(0), [0;-r/2]);%toe 1
gtoe2_f = g_f2_f*G(R(0), [0;-r/2]);%toe 2

hold on
%Torso
line([g_T_f(1,3);gh_f(1,3)],[g_T_f(2,3);gh_f(2,3)]);
%Femur1
line([gh_f(1,3);gtoe1_f(1,3)],[gh_f(2,3);gtoe1_f(2,3)]);
% %Femur2
line([gh_f(1,3);gtoe2_f(1,3)],[gh_f(2,3);gtoe2_f(2,3)]);

    strmin = ['t = ',num2str(round(t*100)/100)];
    text(-0.5,1.2,strmin);

axis([-2,5,-2,2])
axis equal
end

