clear;
clc;
close all;

load('mst1.txt');
load('mst2.txt');
%load('slv.txt');


fX1 = mst1(:,1);
fY1 = mst1(:,2);
fZ1 = mst1(:,3);

EX11_ref = mst1(:,4);
EY11_ref = mst1(:,5);
EZ11_ref = mst1(:,6);

EX11 = mst1(:,7);
EY11 = mst1(:,8);
EZ11 = mst1(:,9);

EX12_ref = mst1(:,10);
EY12_ref = mst1(:,11);
EZ12_ref = mst1(:,12);

EX12 = mst1(:,13);
EY12 = mst1(:,14);
EZ12 = mst1(:,15);

fX2 = mst2(:,1);
fY2 = mst2(:,2);
fZ2 = mst2(:,3);

EX21_ref = mst2(:,4);
EY21_ref = mst2(:,5);
EZ21_ref = mst2(:,6);

EX21 = mst2(:,7);
EY21 = mst2(:,8);
EZ21 = mst2(:,9);

EX22_ref = mst2(:,10);
EY22_ref = mst2(:,11);
EZ22_ref = mst2(:,12);

EX22 = mst2(:,13);
EY22 = mst2(:,14);
EZ22 = mst2(:,15);

%{
fX3 = slv(:,1);
fY3 = slv(:,2);
fZ3 = slv(:,3);

EX31_ref = slv(:,4);
EY31_ref = slv(:,5);
EZ31_ref = slv(:,6);

EX31 = slv(:,7);
EY31 = slv(:,8);
EZ31 = slv(:,9);

EX32_ref = slv(:,10);
EY32_ref = slv(:,11);
EZ32_ref = slv(:,12);

EX32 = slv(:,13);
EY32 = slv(:,14);
EZ32 = slv(:,15);

%}
figure(1);
plot(fX1,'r');
hold on;
plot(fY1,'k');
hold on;
plot(fZ1,'b');
grid;

figure(2);
subplot(3,2,1);
plot(EX11_ref,'r');
hold on;
plot(-EX11,'k');
legend('ref','out');
grid;

subplot(3,2,2);
plot(EY11_ref,'r');
hold on;
plot(-EY11,'k');
legend('ref','out');
grid;

subplot(3,2,3);
plot(EZ11_ref,'r');
hold on;
plot(-EZ11,'k');
legend('ref','out');
grid;

subplot(3,2,4);
plot(EX12_ref,'r');
hold on;
plot(-EX12,'k');
legend('ref','out');
grid;

subplot(3,2,5);
plot(EY12_ref,'r');
hold on;
plot(-EY12,'k');
legend('ref','out');
grid;

subplot(3,2,6);
plot(EZ12_ref,'r');
hold on;
plot(-EZ12,'k');
legend('ref','out');
grid;


figure(3);
plot(fX2,'r');
hold on;
plot(fY2,'k');
hold on;
plot(fZ2,'b');
legend('ref','out');
grid;

figure(4);
subplot(3,2,1);
plot(EX21_ref,'r');
hold on;
plot(-EX21,'k');
legend('ref','out');
grid;

subplot(3,2,2);
plot(EY21_ref,'r');
hold on;
plot(-EY21,'k');
legend('ref','out');
grid;

subplot(3,2,3);
plot(EZ21_ref,'r');
hold on;
plot(-EZ21,'k');
legend('ref','out');
grid;

subplot(3,2,4);
plot(EX22_ref,'r');
hold on;
plot(-EX22,'k');
legend('ref','out');
grid;

subplot(3,2,5);
plot(EY22_ref,'r');
hold on;
plot(-EY22,'k');
legend('ref','out');
grid;

subplot(3,2,6);
plot(EZ22_ref,'r');
hold on;
plot(-EZ22,'k');
legend('ref','out');
grid;


%{
figure(5);
plot(fX3,'r');
hold on;
plot(fY3,'k');
hold on;
plot(fZ3,'b');
legend('ref','out');
grid;

figure(6);
subplot(3,2,1);
plot(EX31_ref,'r');
hold on;
plot(-EX31,'k');
legend('ref','out');
grid;

subplot(3,2,2);
plot(EY31_ref,'r');
hold on;
plot(-EY31,'k');
legend('ref','out');
grid;

subplot(3,2,3);
plot(EZ31_ref,'r');
hold on;
plot(-EZ31,'k');
legend('ref','out');
grid;

subplot(3,2,4);
plot(EX32_ref,'r');
hold on;
plot(-EX32,'k');
legend('ref','out');
grid;

subplot(3,2,5);
plot(EY32_ref,'r');
hold on;
plot(-EY32,'k');
legend('ref','out');
grid;

subplot(3,2,6);
plot(EZ32_ref,'r');
hold on;
plot(-EZ32,'k');
legend('ref','out');
grid;
%}