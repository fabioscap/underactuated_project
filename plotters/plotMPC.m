close all; clc; clear;

pos = load("../data/desired.comPos");
zmp = load("../data/desired.zmpPos");
left = load("../data/desired.leftFoot.pos");
right = load("../data/desired.rightFoot.pos");

cleft = load("../data/current.leftFoot.pos");
cright = load("../data/current.rightFoot.pos");
cpos = load("../data/current.comPos");

comvel = load("../data/desired.comVel");
comvel_curr = load("../data/current.comVel");


figure; clf; hold on;
plot(left(:,3))
plot(cleft(:,3))
% plot(right(:,3))
% plot(cright(:,3))

figure; clf; hold on;
plot(pos(:,1))
plot(cpos(:,1))
legend('comp','meas')

figure; clf; hold on;
plot(comvel(:,1))
plot(comvel_curr(:,1))
legend('comp','meas')

figure; clf; hold on;
plot(pos(:,2))
plot(cpos(:,2))
plot(zmp(:,2))
legend('com','com meas','zmp')