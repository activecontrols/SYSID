clear;close all

%% user input
durationBefore = 1; % desired time before throttle on to save of data. set to 0 for none (s)
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

indChanges =  find([1,diff(vane')] ~= 0); % indices of change, either to a vane angle or back to zero
vaneStarts = indChanges(2:2:end); % first index of each test
vaneStartsOG = vaneStarts; % duplicate before change for plot
vaneEnds = vaneStarts + 13;% indChanges(3:2:end)-1; % last index of each test
% ok this is a little dirty but they were all 11-13 long.
% this way they are all the same length

for i = 1:numThrottleLevels
    for j = 1:numVaneAngles
        trueTestNum = (i-1)*numVaneAngles + j;
        % I didn't know the best way to deal w the duplicate at T=10,
        % VA=-14. this will keep the values on track w the testnum plot.
        trueTestNum = trueTestNum + (trueTestNum>23); 
    
        if durationBefore ~= 0 % offset start or end
            [~, idx] = min(abs(time - (time(vaneStarts(trueTestNum)) - durationBefore)));
            vaneStarts(trueTestNum) = idx;
        end
        if durationAfter ~= 0
            [~, idx] = min(abs(time - (time(vaneEnds(trueTestNum)) + durationAfter)));
            vaneEnds(trueTestNum) = idx;
        end

        % data(i,j,:) = rawData(vaneStarts(trueTestNum):vaneEnds(trueTestNum),6); % this 3d wont work :*(
        dataTime(trueTestNum,:) =  rawData(vaneStarts(trueTestNum):vaneEnds(trueTestNum),2);        
        dataRoll(trueTestNum,:) =  rawData(vaneStarts(trueTestNum):vaneEnds(trueTestNum),6);
        dataPitch(trueTestNum,:) =  rawData(vaneStarts(trueTestNum):vaneEnds(trueTestNum),7);
        dataYaw(trueTestNum,:) =  rawData(vaneStarts(trueTestNum):vaneEnds(trueTestNum),8);
        dataAccelX(trueTestNum,:) =  rawData(vaneStarts(trueTestNum):vaneEnds(trueTestNum),9);
        dataGX(trueTestNum,:) =  rawData(vaneStarts(trueTestNum):vaneEnds(trueTestNum),12);
        dataMagX(trueTestNum,:) =  rawData(vaneStarts(trueTestNum):vaneEnds(trueTestNum),15);
        %% export more columns as desired

    end
end


%% Export to new csv(s)
% 

%% plots
if plotYorN == "y" || plotYorN == "Y"
    plot(1:length(vaneStarts), vane(vaneStartsOG), ".")
    title("sanity check for index of starts. use location on plot to infer throttle")
    xlabel("true test num")
    ylabel("vane angle")
    
    fakeTime = linspace(-durationBefore,2+durationAfter,length(dataRoll(1,:)) ); % give them all universal time index
    figure
    plot(fakeTime, dataRoll(1:25,:))
    title("Roll Data for 10% Throttle Tests")
    xlabel("Time from Throttle On")

    figure
    plot(fakeTime, dataPitch(26:49,:))
    title("Pitch Data for 30% Throttle Tests")
    xlabel("Time from Throttle On")

    figure
    plot(fakeTime, dataAccelX(50:73,:))
    title("AccelX Data for 50% Throttle Tests")
    xlabel("Time from Throttle On")

    figure
    plot(fakeTime, dataGX(74:97,:))
    title("gx Data for 70% Throttle Tests")
    xlabel("Time from Throttle On")

    figure
    plot(fakeTime, dataMagX(74:97,:))
    title("magX Data for 70% Throttle Tests")
    xlabel("Time from Throttle On")
end
