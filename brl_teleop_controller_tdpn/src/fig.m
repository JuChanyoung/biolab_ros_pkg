clear;
close all;

load('mst.txt');
load('slv.txt');


mst_refX = mst(:,1);
mst_outX = mst(:,2);

slv_refX = slv(:,1);
slv_outX = slv(:,2);


mst_refY = mst(:,3);
mst_outY = mst(:,4);

slv_refY = slv(:,3);
slv_outY = slv(:,4);


mst_refZ = mst(:,5);
mst_outZ = mst(:,6);

slv_refZ = slv(:,5);
slv_outZ = slv(:,6);

figure(1);
subplot(3,2,1);
plot(mst_refX,'r');
hold on;
plot(-mst_outX,'b');
grid;
legend('Refef','Out');
title('Master');

subplot(3,2,2);
plot(slv_refX,'r');
hold on;
plot(-slv_outX,'b');
grid on;
legend('Ref','Out');
title('Slave');


subplot(3,2,3);
plot(mst_refY,'r');
hold on;
plot(-mst_outY,'b');
grid on;
legend('Refef','Out');
title('Master');

subplot(3,2,4);
plot(slv_refY,'r');
hold on;
plot(-slv_outY,'b');
grid on;
legend('Ref','Out');
title('Slave');

subplot(3,2,5);
plot(mst_refZ,'r');
hold on;
plot(-mst_outZ,'b');
grid on;
legend('Refef','Out');
title('Master');

subplot(3,2,6);
plot(slv_refZ,'r');
hold on;
plot(-slv_outZ,'b');
grid;
legend('Ref','Out');
title('Slave');

