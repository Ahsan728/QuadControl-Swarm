clear all
close all

a = dlmread(['./martin_6D.tsv'],'\t',14,0);
t = a(:,2);
x = a(:,3)*1e-3;
y = a(:,4)*1e-3;
z = a(:,5)*1e-3;
deltat = (t(2) - t(1));

vx = diff(x)/deltat;
vy = diff(y)/deltat;
vz = diff(z)/deltat;

b = dlmread('./experiment_data.csv',',',1,0);
t2= (b(:,1)-2802227)/15200*10;
%t2 = 0:0.1:92*0.1;
x2 = b(:,2);
y2 = b(:,3);
z2 = b(:,4);
vx2 = b(:,5);
vy2 = b(:,6);
vz2 = b(:,7);

plot(t(1:end-1),vx);
hold on
plot(t2+4,vx2);
%ylim([-0.5, 0.5])

figure
plot(t(1:end-1),vy);
hold on
plot(t2+4,vy2);
%ylim([-0.5, 0.5])

figure
plot(t(1:end-1),vz);
hold on
plot(t2+4,vz2);
%ylim([-0.5, 0.5])

