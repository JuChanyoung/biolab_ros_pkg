clear;
close all;

load('mst.txt');
load('slv.txt');


mstEp = mst(:,1);
mstEn = mst(:,2);

slvEp = slv(:,1);
slvEn = slv(:,2);

figure(1);
subplot(2,1,1);
%plot(mstEp,'r');
hold on;
plot(-mstEn,'b');
grid;
legend('Ep','En');
title('Master');

subplot(2,1,2);
plot(slvEp,'r');
hold on;
plot(-slvEn,'b');
grid;
legend('Ep','En');
title('Slave');