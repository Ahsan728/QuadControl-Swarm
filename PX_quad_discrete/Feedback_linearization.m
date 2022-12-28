
clc
clear
close all
%% Input constraints
g=9.81;
Tmax = 2 * g;
phimax = 10*pi/180;
thetamax = 10*pi/180;

%% Simulation setup
X_init = [0;0;0;0;0;0];
v=[0;0;0];
X = X_init;
psi = 0;
u=[0;0;0];
%% Control design
Ts= 0.1; %sampling time
Ad=[eye(3), Ts*eye(3);
   zeros(3) eye(3)];

Bd=[0.5*Ts^2*eye(3);
   Ts*eye(3)];

K = [eye(3)*-2.5,eye(3)*-1.5];
%% Reference
Xref = zeros(6,100)+[0;2;3;0;0;0];
vref=zeros(3,100);

%% Simulation
for k = 1:100
    %Compute the controller v
    v(:,k) = vref(:,k) + K * (X(:,k)-Xref(:,k));
    %Compute the real control u
    v1 = v(1,k);v2 = v(2,k);v3 = v(3,k);
    Thrust= sat(sqrt(v1^2+v2^2+(v3+g)^2),0,Tmax);
    phi   = sat(asin((v1*sin(psi) - v2*cos(psi))/(Thrust)),-phimax,phimax);
    theta = sat(atan((v1*cos(psi) + v2*sin(psi))/(v3+g)),-thetamax,thetamax);
    u(:,k)=[Thrust,phi,theta];
    % update the position for the drone
    X(:,k+1)= Drone_translational_norm(psi,X(:,k),u(:,k),Ts);
end

%%
figure
plot3(X(1,:),X(2,:),X(3,:))
grid on

%%

function y=sat(x,xmin,xmax)
if x<xmin
    y=xmin;
else
    if x>xmax
        y=xmax;
    else
        y=x;
    end
end
end
