function drone_plot_w(x,y,z,phi,theta,psi,d)
h=d/10;
scatter3(x,y,z,25,'filled','k')
R=[cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
   cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
   -sin(theta),   sin(phi)*cos(theta),  cos(phi)*cos(theta)];

x1=d*R*[1 0 0]' +[x,y,z]';
x2=d*R*[-1 0 0]'+[x,y,z]';
x3=d*R*[0 1 0]' +[x,y,z]';
x4=d*R*[0 -1 0]'+[x,y,z]';

% xw1=R*([1 0 0]'+[0;0;h])+[x,y,z]';
% xw2=R*([-1 0 0]'+[0;0;h])+[x,y,z]';
% xw3=R*([0 1 0]'+[0;0;h])+[x,y,z]';
% xw4=R*([1 -1 0]'+[0;0;h])+[x,y,z]';

xw=R*[0;0;h];
xw1=xw+x1;
xw2=xw+x2;
xw3=xw+x3;
xw4=xw+x4;

plot3([x1(1),x2(1)],[x1(2),x2(2)],[x1(3),x2(3)],'b','linewidth',4)
plot3([x3(1),x4(1)],[x3(2),x4(2)],[x3(3),x4(3)],'r','linewidth',4)

plot3([x1(1),xw1(1)],[x1(2),xw1(2)],[x1(3),xw1(3)],'k','linewidth',3)
plot3([x2(1),xw2(1)],[x2(2),xw2(2)],[x2(3),xw2(3)],'k','linewidth',3)
plot3([x3(1),xw3(1)],[x3(2),xw3(2)],[x3(3),xw3(3)],'k','linewidth',3)
plot3([x4(1),xw4(1)],[x4(2),xw4(2)],[x4(3),xw4(3)],'k','linewidth',3)

end