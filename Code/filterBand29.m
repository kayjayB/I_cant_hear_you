function Hd = filterBand29
%FILTERBAND29 Returns a discrete-time filter object.

% MATLAB Code
% Generated by MATLAB(R) 9.4 and DSP System Toolbox 9.6.
% Generated on: 13-Jul-2018 17:17:42

% Equiripple Bandpass filter designed using the FIRPM function.

% All frequency values are in Hz.
Fs = 40000;  % Sampling Frequency

Fstop1 = 630;             % First Stopband Frequency
Fpass1 = 708;             % First Passband Frequency
Fpass2 = 891;             % Second Passband Frequency
Fstop2 = 1000;            % Second Stopband Frequency
Dstop1 = 0.001;           % First Stopband Attenuation
Dpass  = 0.057501127785;  % Passband Ripple
Dstop2 = 0.001;           % Second Stopband Attenuation
dens   = 20;              % Density Factor

% Calculate the order from the parameters using FIRPMORD.
[N, Fo, Ao, W] = firpmord([Fstop1 Fpass1 Fpass2 Fstop2]/(Fs/2), [0 1 ...
    0], [Dstop1 Dpass Dstop2]);

% Calculate the coefficients using the FIRPM function.
b  = firpm(N, Fo, Ao, W, {dens});
Hd = dsp.FIRFilter( ...
    'Numerator', b);

% [EOF]
