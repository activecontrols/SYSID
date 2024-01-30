function new_rpm = fillRPM(rpm)
    if isnan(rpm(1))
        rpm(1) = 0;
    end
    currentRPM = rpm(1);
    for i = 2:length(rpm)
        if isnan(rpm(i))
            rpm(i) = currentRPM;
        else
            currentRPM = rpm(i);
        end
    end
    new_rpm = rpm;
end