close; clc; clear;


% pos_cur = load("../data/current.leftFoot.ang_pos");
pos_des = load("../data/desired.leftFoot.ang_pos");

% vel_cur = load("../data/current.rightFoot.vel");
vel_des = load("../data/desired.leftFoot.ang_vel");

% acc_cur = load("../data/current.rightFoot.acc");
acc_des = load("../data/desired.leftFoot.ang_acc");

figure; clf; hold on;
plot(pos_des(:,2))
legend

figure; clf; hold on;
plot(vel_des(:,2))
plot(diff(pos_des(:,2))/0.01)
legend

figure; clf; hold on;
plot(acc_des(:,2))
plot(diff(vel_des(:,2))/0.01)
legend