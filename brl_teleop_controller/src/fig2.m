clear;
close all;

load('slv.txt');

fX1 = slv(:,1);
fY1 = slv(:,2);
fZ1 = slv(:,3);

fX2 = slv(:,4);
fY2 = slv(:,5);
fZ2 = slv(:,6);


figure(1);
subplot(3,1,1);
plot(fX1,'r');
hold on;
plot(fX2,'b');
grid;

subplot(3,1,2);
plot(fY1,'r');
hold on;
plot(fY2,'b');
grid;

subplot(3,1,3);
plot(fZ1,'r');
hold on;
plot(fZ2,'b');
grid;
