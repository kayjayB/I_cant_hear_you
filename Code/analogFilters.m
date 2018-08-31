d3kHz = fdesign.bandpass('N,F3dB1,F3dB2',2,2.8e3,3.5e3,4e4);
d6kHz = fdesign.bandpass('N,F3dB1,F3dB2',2,5.6e3,7e3,4e4);
Hd3kHz = design(d3kHz,'butter');
Hd6kHz = design(d6kHz,'butter');
fvtool(Hd3kHz,Hd6kHz)
legend('2.8-3.5 kHz','5.6-7 kHz')

% [A,B,C,D] = butter(2,[2800 3500]/8000);
% sos = ss2sos(A,B,C,D);
% fvtool(sos)