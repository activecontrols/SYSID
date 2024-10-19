function [f,amp] = myfft(data, tsamp)

% Script is FULLY copied from "A simple MEMS gyro model using MATLAB / Simulink"
% https://www.youtube.com/watch?v=P1OEoA70YJo&list=WL&index=42 @ 9:29

Fs = 1/tsamp; % sample frequency
T = tsamp; % sample time
L = length(data); % data length

x = data;

NFFT =  2^nextpow2(L);  % next pwoer of 2 from length of x
Y = fft(x,NFFT)/L; % FFT
f = Fs/2*linspace(0,1,NFFT/2+1); % generatre frequency vector

% plot single-sided amplitude spectrum
amp = 2*abs(Y(1:NFFT/2+1));
loglog(f,amp)
title("Single-Sided Amplitude Spectrum of data(t)")
xlabel("Frequency (Hz)")
ylabel("|Data(f)|")
end