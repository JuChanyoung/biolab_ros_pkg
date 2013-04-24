clear;
clc;
close all;

load('mst1.txt');
load('mst2.txt');
load('slv.txt');


fX1 = mst1(:,1);
fY1 = mst1(:,2);
fZ1 = mst1(:,3);

X1 = mst1(:,4);
Y1 = mst1(:,5);
Z1 = mst1(:,6);

EX11_ref = mst1(:,7);
EY11_ref = mst1(:,8);
EZ11_ref = mst1(:,9);

EX11 = mst1(:,10);
EY11 = mst1(:,11);
EZ11 = mst1(:,12);

EX12_ref = mst1(:,13);
EY12_ref = mst1(:,14);
EZ12_ref = mst1(:,15);

EX12 = mst1(:,16);
EY12 = mst1(:,17);
EZ12 = mst1(:,18);


fX2 = mst2(:,1);
fY2 = mst2(:,2);
fZ2 = mst2(:,3);

X2 = mst2(:,4);
Y2 = mst2(:,5);
Z2 = mst2(:,6);


EX21_ref = mst2(:,7);
EY21_ref = mst2(:,8);
EZ21_ref = mst2(:,9);

EX21 = mst2(:,10);
EY21 = mst2(:,11);
EZ21 = mst2(:,12);

EX22_ref = mst2(:,13);
EY22_ref = mst2(:,14);
EZ22_ref = mst2(:,15);

EX22 = mst2(:,16);
EY22 = mst2(:,17);
EZ22 = mst2(:,18);


fX3 = slv(:,1);
fY3 = slv(:,2);
fZ3 = slv(:,3);

X3 = slv(:,4);
Y3 = slv(:,5);
Z3 = slv(:,6);

EX31_ref = slv(:,7);
EY31_ref = slv(:,8);
EZ31_ref = slv(:,9);

EX31 = slv(:,10);
EY31 = slv(:,11);
EZ31 = slv(:,12);

EX32_ref = slv(:,13);
EY32_ref = slv(:,14);
EZ32_ref = slv(:,15);

EX32 = slv(:,16);
EY32 = slv(:,17);
EZ32 = slv(:,18);


figure(1);
plot(fY1,'r');
hold on;
plot(fY2,'b');
hold on;
plot(fY3,'k');
legend('Master1','Master2','Slave');
title('Force tracking')
grid;

figure(2);
plot(Y1,'r');
hold on;
plot(Y2,'b');
hold on;
plot(Y3,'k');
legend('Master1','Master2','Slave')
title('Position tracking')
grid;