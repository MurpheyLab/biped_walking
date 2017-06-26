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
line([g_T_f(1,3);gh_f(1,3)],[g_T_f(2,3);gh_f(2,3)],'LineWidth',2);
%Femur1
line([gh_f(1,3);gtoe1_f(1,3)],[gh_f(2,3);gtoe1_f(2,3)],'LineWidth',2);
% %Femur2
line([gh_f(1,3);gtoe2_f(1,3)],[gh_f(2,3);gtoe2_f(2,3)],'LineWidth',2);

%Masses
masses = [x(4),x(5);g_T_f(1,3),g_T_f(2,3);g_f1_f(1,3),g_f1_f(2,3);g_f2_f(1,3),g_f2_f(2,3)];
plot(masses(1:2,1),masses(1:2,2),'.','markersize',25,'Color','r')
plot(masses(3:4,1),masses(3:4,2),'.','markersize',20,'Color','r')

    strmin = ['t = ',num2str(round(t*100)/100)];
   text(0,1.7,strmin);

    %Floor
    line([-1;9.5],[0;0],'Color','k','LineWidth',2)
    
            
%     th = 0:0.1:2*pi;
%     
%    plot(0.2*(t)+0.05*cos(th), 1+0.05*sin(th))
    
    axis equal
axis([-0.5,9.1,-0.5,2])
xlabel('x(m)')
ylabel('y(m)')

 set(gca,'FontSize',15)
    set(findall(gcf,'type','text'),'fontSize',15)
    
title('Walking with iSAC','FontSize', 20)
 set(gcf,'Position',[1 1 1800 750])%set dimensions of figure
set(gca,'FontName','Times New Roman');
%FOR GAIT INITIATION FIGURE
% title('Gait Initiation','FontSize', 20,'fontWeight','bold')
%     set(gcf,'Position',[1 1 900 750])%set dimensions of figure
    



end

