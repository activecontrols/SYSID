% clear;close all % don't or else you'll clear simulink. clear manually if needed
%% This script is based of parse_testing_data.m in the roll test data folder. a lot of stuff is gutted out tho
%% it goes together with the simulink model (should be in the same folder)
% data sample
% segment,time,throttle,vane,rpm,roll,pitch,yaw,accelx,accely,accelz,gx,gy,gz,magx,magy,magz,encoderGamma,encoderBeta,
% 0,5.611000,0.000000,0.000000,nan,-4.254608,-6.229400,28.223263,-0.502493,0.039482,9.923634,-0.031714,-0.039384,0.125051,-86.831543,-22.515490,-77.255043,0.000000,0.000000,
rawData = readmatrix('log.csv');
time = rawData(:,2); % seconds from start of test

accelX = rawData(:,9); % accelerometer data (m/s^2)
accelY = rawData(:,10);
accelZ = rawData(:,11);

aXbias = mean(accelX); % accelerometer data (m/s^2)
aYbias = mean(accelY);
aZbias = mean(accelZ);

gx = rawData(:,12); % gyrometer data (rad/s)
gy = rawData(:,13);
gz = rawData(:,14);

magX = rawData(:,15); % magnetometer data (microtesla)
magY = rawData(:,16);
magZ = rawData(:,17);

axMean = mean(accelX);
ayMean = mean(accelY);
azMean = mean(accelZ);

gxMean = mean(gx);
gyMean = mean(gy);
gzMean = mean(gz);

gxStd = std(gx);
gyStd = std(gy);
gzStd = std(gz);

%% plots just from raw data
figure
subplot(3,1,1)
plot(time,gx)
title("gx raw data")
xlabel("time (s)")
ylabel("rotation (rad/s)")
subplot(3,1,2)
plot(time,gy)
title("gy raw data")
xlabel("time (s)")
ylabel("rotation (rad/s)")
subplot(3,1,3)
plot(time,gz)
title("gz raw data")
xlabel("time (s)")
ylabel("rotation (rad/s)")

figure
subplot(3,1,1)
histogram(gx)
title(sprintf("gx- mean: %f std: %f", gxMean, gxStd))
xlabel("rad/s")
ylabel("freq")
subplot(3,1,2)
histogram(gy)
title(sprintf("gy- mean: %f std: %f", gyMean, gyStd))
xlabel("rad/s")
ylabel("freq")
subplot(3,1,3)
histogram(gz)
title(sprintf("gz- mean: %f std: %f", gzMean, gzStd))
xlabel("rad/s")
ylabel("freq")

%% Plots using simulink

function compareData(actual, model, actualTime, modelTime)

    figure
    plot(actualTime, actual, modelTime, model)
    legend("real data","simulink model")
    xlabel("time (s)")
    ylabel("rotation (rad/s)")
    
    actualMean = mean(actual);

    figure
    [f,amp] = myfft(actual-actualMean, 16);
    hold on
    [fsim,ampsim] = myfft(model-actualMean, 16);
    
    figure
    [tau,AVAR] = allan(actualTime, actual-actualMean);
    hold on
    [tausim,AVARsim] = allan(modelTime, model-actualMean);
    title("Allan Variance Plot")
    legend("real data","simulink model")
    xlabel("tau")
    ylabel("Allan Variance")
    
    figure
    histogram(actual, "BinWidth",.0035)
    hold on
    histogram(model,"BinWidth",.0035)
    xlim([-0.1, 0.1]);
    title("Histogram of model vs raw")
    legend("real data","simulink model")
    xlabel("rad/s")
    ylabel("freq")

    calcRateNoiseDensity = AVAR(16);
    calcRateNoiseDensitysim = AVARsim(16);

    fprintf("tau check: %f, %f, calculated RND of sensor: %f, calcRND of sim: %f\n", tau(16),tausim(16), AVAR(16), AVARsim(16))
    fprintf("mean of sensor: %f, mean of sim: %f\n", actualMean, mean(model))
    fprintf("std of sensor: %f, std of sim: %f\n", std(actual), std(model))
    fprintf("ideal mean is 0. ideal RND is .0038\n")
end

compareData(accelZ, out.az, time, out.tout);

% tau(16) at this point, tau is closest to 1
%calcRateNoiseDensity = AVAR(16);
%calcRateNoiseDensitysim = AVARsim(16);
% ans = .0044. the data sheet says it should be .0038
%fprintf("tau check: %f, %f, calculated RND of sensor: %f, calcRND of sim: %f\n", tau(16),tausim(16), AVAR(16), AVARsim(16))
%fprintf("mean of sensor: %f, mean of sim: %f\n", gzMean, mean(out.gy))
%fprintf("std of sensor: %f, std of sim: %f\n", gzStd, std(out.gy))
%fprintf("ideal mean is 0. ideal RND is .0038\n")

%% what I did to make the model
% 1. input the static bias into model
% 2. put in expected RND into white noise block > noise power
% 3. leave scale factor
% 4. run simulink, then run this script
% 5. enter raw data RND for model, tweak scale factor to match raw

%% things to do
% test the three axis gyro from aerospace tooblock set
% try running cleaned data through noise model and see how it looks (could
%    use roll test)
% get longer raw data sample
% repeat process for gx,gy mag, and accel
% I noticed messing w seed fucked RND