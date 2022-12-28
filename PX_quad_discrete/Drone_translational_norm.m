function y=Drone_translational_norm(psi,xk,uk,Ts)
% RK4:= fd(x,u)
T=uk(1);phi=uk(2);tt=uk(3);
g=9.81;
A=[eye(3), Ts*eye(3);
   zeros(3) eye(3)];

h=[0.5*Ts^2*eye(3);
   Ts*eye(3)];

hh1=T*(cos(phi)*sin(tt)*cos(psi) + sin(phi)*sin(psi))  ;
hh2=T*(cos(phi)*sin(tt)*sin(psi) - sin(phi)*cos(psi));
hh3= -g +T*cos(phi)*cos(tt);

y=A*xk+ h*[hh1 hh2 hh3]';
end