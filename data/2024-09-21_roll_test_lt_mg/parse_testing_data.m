clear;close all

%% user input
durationBefore = 2; % desired time before throttle on to save of data. set to 0 for none (s)
durationAfter = 2; % desired time after throttle off to save of data. set to 0 for none (s)
plotYorN = "y"; % if you want to show plots or nah (input "y" or "n")
% exportYorN = "n"; % not done, not sure if useful

%% the rest
% data sample
% segment,time,throttle,vane,rpm,roll,pitch,yaw,accelx,accely,accelz,gx,gy,gz,magx,magy,magz,encoderGamma,encoderBeta,
% 0,5.611000,0.000000,0.000000,nan,-4.254608,-6.229400,28.223263,-0.502493,0.039482,9.923634,-0.031714,-0.039384,0.125051,-86.831543,-22.515490,-77.255043,0.000000,0.000000,
rawData = readmatrix('log.csv');
time = rawData(:,2); % seconds from start of test
throttle = rawData(:,3); % values from 0->100
vane = rawData(:,4); % angle in DEGREES

throttleLevels = unique(throttle);
vaneAngles = unique(vane);
numThrottleLevels = length(throttleLevels)-1; % -1 accts for zero
numVaneAngles = length(vaneAngles)-1;
numTests = numThrottleLevels*numVaneAngles + 1; % -2 to ignore 0 throttle & VA. +1 accts for duplicate


indChanges =  find([1,diff(vane')] ~= 0); % indices of change, either to a vane angle or back to zero
vaneStarts = indChanges(2:2:end); % first index of each test
vaneStartsOG = vaneStarts; % duplicate before change for plot
maxTestLength = max((indChanges(3:2:end)-1)-vaneStarts); % max length in terms of timesteps
vaneEnds = vaneStarts + maxTestLength; % last index of each test. this way they're all same length

rawYaw = rawData(:,8);
yawOverflower = zeros(length(rawYaw),1); % this will keep track of when yaw crosses over -180 or +180 border
    % angle rollover for yaw; can repeat for roll or pitch also
    for j = 2:length(rawData) 
        if rawYaw(j) > 140 && rawYaw(j-1) < -140
            yawOverflower(j:end) = yawOverflower(j:end) - 360;
        elseif rawYaw(j) <-140 && rawYaw(j-1) > 140
            yawOverflower(j:end) = yawOverflower(j:end) + 360;
        end
    end
rawYawAbs = rawYaw + yawOverflower;


for i = 1:numTests
    if durationBefore ~= 0 % offset start or end
        [~, idx] = min(abs(time - (time(vaneStarts(i)) - durationBefore)));
        vaneStarts(i) = idx;
    end
    if durationAfter ~= 0
        [~, idx] = min(abs(time - (time(vaneEnds(i)) + durationAfter)));
        vaneEnds(i) = idx;
    end

    % export more columns as desired
    dataTime(i,:) =  rawData(vaneStarts(i):vaneEnds(i),2);     
    % dataRoll(i,:) =  rawData(vaneStarts(i):vaneEnds(i),6);
    % dataPitch(i,:) =  rawData(vaneStarts(i):vaneEnds(i),7);
    dataYaw(i,:) =  rawData(vaneStarts(i):vaneEnds(i),8);
    dataYawAbs(i,:) = rawYawAbs(vaneStarts(i):vaneEnds(i));
    dataAccelX(i,:) =  rawData(vaneStarts(i):vaneEnds(i),9);
    dataGZ(i,:) =  rawData(vaneStarts(i):vaneEnds(i),14);
end

dataYawRate = gradient(dataYawAbs) ./ gradient(dataTime);
%% Export to new csv(s)
% 

%% plots
% mb there's def a better way to code but also I
% kinda lazy and also had exams and hw
if plotYorN == "y" || plotYorN == "Y"
    figure % sanity check thing
    plot(1:length(vaneStarts), vane(vaneStartsOG), ".") 
    title("sanity check for index of starts. use location on plot to infer throttle")
    xlabel("true test num")
    ylabel("vane angle")
    
    figure
    n=60;
    fakeTime = linspace(-durationBefore,2+durationAfter,length(dataYaw(1,:)) ); % give them all universal time index
    values = [-15 -14 -13 -12 -11 -10 -7 -5 -4 -3 -2 -1 1 2 3 4 5 7 10 11 12 13 14 15]; % for legend for vane anglse
    plot(fakeTime, dataYawAbs(n,:))
    hold on
    plot(fakeTime, gradient(dataYawAbs(n,:)) ./ gradient (dataTime(n,:)) )
    plot(fakeTime, dataGZ(n,:)*180/pi)
    legend("Filtered yaw", "Slope of filtered yaw", "gyroscope z roll rate")
    xlabel("Time from Throttle On")
    ylabel("Yaw (degrees)/Yaw rate (degrees/s)")
    title("Yaw, Yaw rate, gyro yaw rate")

    % figure % plot yaw data
    % plotGradient(fakeTime, dataYaw(1:25,:))
    % title("Yaw Data for 10% Throttle Tests")
    % xlabel("Time from Throttle On")
    % ylabel("Yaw (degrees)")
    % legend(string(values))
    % 
    % figure
    % plotGradient(fakeTime, dataYaw(26:49,:))
    % title("Yaw Data for 30% Throttle Tests")
    % xlabel("Time from Throttle On")
    % ylabel("Yaw (degrees)")
    % legend(string(values))
    % 
    % figure
    % plotGradient(fakeTime, dataYaw([50 54 56 61],:))
    % title("Yaw Data for 50% Throttle Tests")
    % xlabel("Time from Throttle On")
    % ylabel("Yaw (degrees)")
    % legend("1", "5", "10", "15")
    % 
    % figure
    % plotGradient(fakeTime, dataYaw([74 78 80 85],:))
    % title("Yaw Data for 70% Throttle Tests")
    % xlabel("Time from Throttle On")
    % ylabel("Yaw (degrees)")
    % legend("1", "5", "10", "15")

    figure % plot yaw RATE data
    plotGradient(fakeTime, dataYawRate([1 5 7 12],:))
    title("Yaw Rate Data for 10% Throttle Tests")
    xlabel("Time from Throttle On")
    ylabel("Yaw Rate (deg/s)")
     legend("1", "5", "10", "15")

    figure
    subplot(1,2,1)
    plotGradient(fakeTime, dataYawRate([26 30 32 37],:))
    title("Yaw Rate Data for 30% Throttle Tests")
    xlabel("Time from Throttle On")
    ylabel("Yaw Rate (deg/s)")
     legend("1", "5", "10", "15")

    subplot(1,2,2)
    plotGradient(fakeTime, dataYawRate([49 44 42 38 ],:))
    title("Yaw Rate Data for 30% Throttle Tests")
    xlabel("Time from Throttle On")
    ylabel("Yaw Rate (deg/s)")
     legend("-15","-10","-5","-1")

    figure
    plotGradient(fakeTime, dataYawRate([50 54 56 61],:))
    title("Yaw Rate Data for 50% Throttle Tests")
    xlabel("Time from Throttle On")
    ylabel("Yaw Rate (deg/s)")
    legend("1", "5", "10", "15")

    figure
    plot(fakeTime, dataYawRate([97 92 90 56 74 78 80 85],:))
    title("Yaw Rate Data for 70% Throttle Tests")
    xlabel("Time from Throttle On")
    ylabel("Yaw Rate (deg/s)")
    legend("-15","-10","-5","-1", "1", "5", "10", "15")


    figure % thrust at different vane angles
    sgtitle("Varying Thrust at Positive Vane Angles")
    subplot(2,2,1)
    plotGradient(fakeTime, dataYawRate([1 26 50 74],:))
    title("1\circ, All Thrust Levels")
    xlabel("Time from Throttle On")
    ylabel("Yaw Rate (deg/s)")
    legend("10%", "30%","50%", "70%")

    subplot(2,2,2)
    plotGradient(fakeTime, dataYawRate([5 30 54 78],:))
    title("5\circ, All Thrust Levels")
    xlabel("Time from Throttle On")
    ylabel("Yaw Rate (deg/s)")
    legend("10%", "30%","50%", "70%")
    
    subplot(2,2,3)
    plotGradient(fakeTime, dataYawRate([7 32 56 80],:))
    title("10\circ, All Thrust Levels")
    xlabel("Time from Throttle On")
    ylabel("Yaw Rate (deg/s)")
    legend("10%", "30%","50%", "70%")

    subplot(2,2,4)
    plotGradient(fakeTime, dataYawRate([12 37 61 83],:))
    title("15\circ, All Thrust Levels")
    xlabel("Time from Throttle On")
    ylabel("Yaw Rate (deg/s)")
    legend("10%", "30%","50%", "70%")

%{
    figure % same thing but at negative angles
    sgtitle("Varying Thrust at Negative Vane Angles")
    subplot(2,2,1)
    plotGradient(fakeTime, dataYaw([13 38 62 86],:))
    title("-1\circ, All Thrust Levels")
    xlabel("Time from Throttle On")
    ylabel("Yaw (degrees)")
    legend("-10%", "-30%","-50%", "-70%")

    subplot(2,2,2)
    plotGradient(fakeTime, dataYaw([17 42 66 90],:))
    title("-5\circ, All Thrust Levels")
    xlabel("Time from Throttle On")
    ylabel("Yaw (degrees)")
    legend("-10%", "-30%","-50%", "-70%")
    
    subplot(2,2,3)
    plotGradient(fakeTime, dataYaw([19 44 68 92],:))
    title("-10\circ, All Thrust Levels")
    xlabel("Time from Throttle On")
    ylabel("Yaw (degrees)")
    legend("-10%", "-30%","-50%", "-70%")

    subplot(2,2,4)
    plotGradient(fakeTime, dataYaw([25 49 73 97],:))
    title("-15\circ, All Thrust Levels")
    xlabel("Time from Throttle On")
    ylabel("Yaw (degrees)")
    legend("-10%", "-30%","-50%", "-70%")


    figure % misc accelx plot
    plotGradient(fakeTime, dataAccelX(26:49,:))
    title("AccelX Data for 30% Throttle Tests")
    xlabel("Time from Throttle On")
    ylabel("Acceleration (m/s^2)")
%}
    figure % big picture roll data plot, first half
    hold on
    wantedPlot = rawYawAbs;%(1:3300);
    tnew = 1:length(wantedPlot);%length(rawData);
    yValues = linspace(min(min(wantedPlot)), max(max(wantedPlot)), length(wantedPlot));
    indexforLine =  vaneStartsOG([1, 25,26,49]);
    plot(tnew, wantedPlot)
    plot(zeros(length(wantedPlot),1)+indexforLine, yValues , "LineWidth",2.5)
    xlabel("time index")
    ylabel("Yaw (degrees)")
    legend("Test Data","Start of 10% tests", "End of 10% tests", "Start of 30% Tests","End of 30% tests")
end
