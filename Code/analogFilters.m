clc
clear
d3kHz = fdesign.bandpass('N,F3dB1,F3dB2',2,2.8e3,3.5e3,4e4);
d6kHz = fdesign.bandpass('N,F3dB1,F3dB2',2,5.6e3,7e3,4e4);
dLP10kHz = fdesign.lowpass('N,F3db',2,10e3,4e4);
dHP1kHz = fdesign.highpass('N,F3db',2,1e3,4e4);
Hd3kHz = design(d3kHz,'butter');
Hd6kHz = design(d6kHz,'butter');
HdLP10kHz = design(dLP10kHz,'butter');
HdHP1kHz = design(dHP1kHz,'butter');
%%
fvtool(Hd3kHz,Hd6kHz)
legend('2.8-3.5 kHz','5.6-7 kHz')
%%
fvtool(HdLP10kHz)
%%
fvtool(HdHP1kHz)

% [A,B,C,D] = butter(2,[2800 3500]/8000);
% sos = ss2sos(A,B,C,D);
% fvtool(sos)