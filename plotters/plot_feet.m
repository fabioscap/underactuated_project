close; clc; clear;

pos_cur = load("../data/current.rightFoot.pos");
pos_des = load("../data/desired.rightFoot.pos");

vel_cur = load("../data/current.rightFoot.vel");
vel_des = load("../data/desired.rightFoot.vel");

acc_cur = load("../data/current.rightFoot.acc");
acc_des = load("../data/desired.rightFoot.acc");

figure; clf; hold on;
plot(pos_cur(:,3))
plot(pos_des(:,3))

figure; clf; hold on;
plot(vel_cur(:,3))
plot(vel_des(:,3))

figure; clf; hold on;
plot(acc_cur(:,3))
plot(acc_des(:,3))

pos_cur = load("../data/current.comPos");
pos_des = load("../data/desired.comPos");

vel_cur = load("../data/current.comVel");
vel_des = load("../data/desired.comVel");

acc_cur = load("../data/current.comAcc");
acc_des = load("../data/desired.comAcc");

figure; clf; hold on;
plot(pos_cur(:,2))
plot(pos_des(:,2))

figure; clf; hold on;
plot(vel_cur(:,2))
plot(vel_des(:,2))

figure; clf; hold on;
plot(acc_cur(:,2))
plot(acc_des(:,2))
