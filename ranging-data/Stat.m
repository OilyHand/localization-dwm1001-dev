clc, clear, close all

load("ranging_data.mat");

sample = pansLocation4;

pos_x = sample(:,1);
pos_y = sample(:,2);
pos_z = sample(:,3);

mean_x = mean(pos_x);
mean_y = mean(pos_y);
mean_z = mean(pos_z);

std_x = std(pos_x);
std_y = std(pos_y);
std_z = std(pos_z);

figure, histogram(pos_x - mean_x)
title("Distribution of x-axis (\sigma"+sprintf("=%.2f cm)", std_x/10))

figure, histogram(pos_y - mean_y)
title("Distribution of y-axis (\sigma"+sprintf("=%.2f cm)", std_y/10))

figure, histogram(pos_z - mean_z)
title("Distribution of z-axis (\sigma"+sprintf("=%.2f cm)", std_z/10))