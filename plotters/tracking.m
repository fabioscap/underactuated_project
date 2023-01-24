close all; clc; clear;


com = load("../data/current.comPos");
left = load("../data/current.leftFoot.pos");
right = load("../data/current.rightFoot.pos");
figure
hold on
plot(com(:,1))
plot(left(:,1))
plot(right(:,1))

figure
hold on
plot(com(:,2))
plot(left(:,2))
plot(right(:,2))

figure
hold on
plot(com(:,3))
plot(left(:,3))
plot(right(:,3))