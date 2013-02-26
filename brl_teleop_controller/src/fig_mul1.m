clear;
clc;
close all;

load('mst1.txt');
load('mst2.txt');
load('slv.txt');

fX1 = mst1(:,1);
fY1 = mst1(:,2);
fZ1 = mst1(:,3);

fX2 = mst2(:,1);
fY2 = mst2(:,2);
fZ2 = mst2(:,3);

fX3 = slv(:,1);
fY3 = slv(:,2);
fZ3 = slv(:,3);


figure(1);
subplot(3,1,1);
plot(fX1,'r');
hold on;
plot(fX2,'k');
hold on;
plot(fX3,'b');
grid;

subplot(3,1,2);
plot(fY1,'r');
hold on;
plot(fY2,'k');
hold on;
plot(fY3,'b');
grid;

subplot(3,1,3);
plot(fZ1,'r');
hold on;
plot(fZ2,'k');
hold on;
plot(fZ3,'b');
grid;
