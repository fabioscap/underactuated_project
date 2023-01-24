close; clc; clear;


pos_cur = load("../data/current.rightFoot.pos");
pos_des = load("../data/desired.rightFoot.pos");

vel_cur = load("../data/current.rightFoot.vel");
vel_des = load("../data/desired.rightFoot.vel");

acc_cur = load("../data/current.rightFoot.acc");
acc_des = load("../data/desired.rightFoot.acc");

figure; clf; hold on;
plot(pos_des(:,3))
legend

figure; clf; hold on;
plot(vel_des(:,3))
plot(diff(pos_des(:,3))/0.01)
legend

figure; clf; hold on;
plot(acc_des(:,3))
plot(diff(vel_des(:,3))/0.01)
legend