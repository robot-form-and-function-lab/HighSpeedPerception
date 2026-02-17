clc; clear; close all; 
%% This matlab script process hsc camera data 


%% function: process trial, takes the text file and produces speed, uncertainty in speed 


% calculate speed for all trials, for all voltages
t1_21v = calcSpeed('2.1V/trial1_2.1V.txt');
t2_21v = calcSpeed('2.1V/trial2_2.1.txt');
t3_21v = calcSpeed('2.1V/trial3_2.1.txt');

v_avg_21 = (t1_21v+t2_21v+t3_21v)/3; 

t1_23v = calcSpeed('2.3V/trial1_2.3.txt');
t2_23v = calcSpeed('2.3V/trial2_2.3.txt');
t3_23v = calcSpeed('2.3V/trial3_2.3.txt');

v_avg_23 = (t1_23v+t2_23v+t3_23v)/3;

t1_25v = calcSpeed('2.5V/trial1_2.5.txt');
t2_25v = calcSpeed('2.5V/trial2_2.5.txt');
t3_25v = calcSpeed('2.5V/trial3_2.5.txt');

v_avg_25 = (t1_25v+t2_25v+t3_25v)/3;

t1_27v = calcSpeed('2.7V/trial1_2.7.txt');
t2_27v = calcSpeed('2.7V/trial2_2.7.txt');
t3_27v = calcSpeed('2.7V/trial3_2.7.txt');

v_avg_27 = (t1_27v+t2_27v+t3_27v)/3;

t1_29v = calcSpeed('2.9V/trial1_2.9.txt');
t2_29v = calcSpeed('2.9V/trial2_2.9.txt');
t3_29v = calcSpeed('2.9V/trial3_2.9.txt');

v_avg_29 = (t1_29v+t2_29v+t3_29v)/3;

t1_31v = calcSpeed('3.1V/trial1_3.1.txt');
t2_31v = calcSpeed('3.1V/trial2_3.1.txt');
t3_31v = calcSpeed('3.1V/trial3_3.1.txt');

v_avg_31 = (t1_31v+t2_31v+t3_31v)/3;

t1_33v = calcSpeed('3.3V/trial1_3.3.txt');
t2_33v = calcSpeed('3.3V/trial2_3.3.txt');
t3_33v = calcSpeed('3.3V/trial3_3.3.txt');

v_avg_33 = (t1_33v+t2_33v+t3_33v)/3;


t1_35v = calcSpeed('3.5V/trial1_3.5.txt');
t2_35v = calcSpeed('3.5V/trial2_3.5.txt');
t3_35v = calcSpeed('3.5V/trial3_3.5.txt');

v_avg_35 = (t1_35v+t2_35v+t3_35v)/3;

t1_37v = calcSpeed('3.7V/trial1_3.7.txt');
t2_37v = calcSpeed('3.7V/trial2_3.7.txt');
t3_37v = calcSpeed('3.7V/trial3_3.7.txt');

v_avg_37 = (t1_37v+t2_37v+t3_37v)/3;

% collect the results in an array 

v_array = [v_avg_21 v_avg_23 v_avg_25 v_avg_27 v_avg_29 v_avg_31 v_avg_33 v_avg_35 v_avg_37];
voltage_array = [2.1 2.3 2.5 2.7 2.9 3.1 3.3 3.5 3.7];

% determine linear regression 
b = voltage_array\v_array;
newY = b(end,:).*voltage_array;



scatter(voltage_array, v_array)
hold on 
plot(voltage_array,newY)


title("voltage vs speed");
xlabel('voltage (V)');
ylabel('speed (m/s)');


function [v,v_error] = calcSpeed(filename)

% pull in text file
%filename = filename;
delimiterIn = '	';	
headerlinesIn = 2;
A = importdata(filename,delimiterIn,headerlinesIn);
% extract first and final rows
first_frame = A.data(1,4);
first_x = A.data(1,2);
first_y =A.data(1,3);

last_frame = A.data(end,4); 
last_x = A.data(end,2);
last_y = A.data(end,3);



% use x,y to calculate euclidean distance 
first_dist = calcEuclidean(first_x,first_y);
last_dist = calcEuclidean(last_x,last_y); 


% calculate the speed
frame_rate = 5000;
fps_inv = 1/frame_rate;

dt = (last_frame-first_frame)*fps_inv; 
total_distance = last_dist - first_dist;


v = total_distance/dt;

% calculate error 
mnt_uct = 1*10^-6; % uncertainty in measuring tool
v_error = sqrt((fps_inv/dt)^2 + (mnt_uct/total_distance)^2);




end 

function distance = calcEuclidean(x,y)
    distance = sqrt(x^2+y^2);
end 


