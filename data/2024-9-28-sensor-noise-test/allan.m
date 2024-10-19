function [tau,AVAR] = allan(time, data)
% Script is FULLY copied from "A simple MEMS gyro model using MATLAB / Simulink"
% https://www.youtube.com/watch?v=P1OEoA70YJo&list=WL&index=42 @ 10:33

% "calc the number of averaging time bins. this first bin is the smallest
% time sample available, and this time is increased until there is only 9
% averaging bins in the data sample. Note: having more than 9 bins reduceds
% the meaningness of the averaging."
numtaus = floor(length(time)/9);
    for b = 1:numtaus
        tau(b) = time(b+1) - time(1);
    end

    % for each tau, calculate the average variance
    for h = 1:numtaus
        binvar = 0;
        avebin = 0;
        totalbins = floor(time(length(time)) / tau(h));
        lengthbins = floor(length(time)/totalbins);
        for j = 1:totalbins
            avebin(j) = mean(data((lengthbins*(j-1)) + 1:lengthbins*(j)));
        end
        for j = 1:(totalbins-1)
            binvar(j) = (avebin(j+1) - avebin(j))^2;
        end

        AVAR(h) = sqrt(sum(binvar)/(2*(totalbins-1)));
    end
    % plot Allan Variance on loglog scale
    loglog(tau,AVAR)
end