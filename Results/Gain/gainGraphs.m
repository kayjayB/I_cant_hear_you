filename = 'ResultsWithSineWave/gain11.csv';
M1 = csvread(filename, 19,4);

gain11 = M1(:,1);

filename = 'ResultsWithSineWave/gain12.csv';
M2 = csvread(filename, 19,4);

gain12 = M2(:,1);

filename = 'ResultsWithSineWave/gain21.csv';
M3 = csvread(filename, 19,4);

gain21 = M3(:,1);
time = 1:length(gain11);
plot(time, gain11, time, gain12, time, gain21)
legend("Gain 11", "Gain 12", "Gain 21" )

%%
Fs = 55100;
n = length(gain11);
xFourier = fft(gain11)/n; % fourier transform of the data
xFourier = 2*xFourier(1:length(gain11)/2+1); % create a one sided
                                                % frequency spectrum                                     
magnitudeFiltered = 20*log10(abs(xFourier));
freqx = linspace(0,Fs/2,length(gain11)/2+1);

nOriginal=length(gain12); 
originalFourier = fft(gain12)/nOriginal;
originalFourier = 2*originalFourier(1:length(gain12)/2+1); 
magnitudeOriginal = 20*log10(abs(originalFourier));
freqx2 = linspace(0,Fs/2,length(gain12)/2+1);
%cla

nOriginal2=length(gain21); 
originalFourier2 = fft(gain21)/nOriginal2;
originalFourier2 = 2*originalFourier2(1:length(gain21)/2+1); 
magnitudeOriginal2 = 20*log10(abs(originalFourier2));
freqx3 = linspace(0,Fs/2,length(gain21)/2+1);

% pause(0.01)
plot(freqx, magnitudeFiltered,'r');
hold on
plot(freqx2, magnitudeOriginal, 'b');
hold on
plot(freqx3, magnitudeOriginal2, 'g');
xlim([0 8000])
legend("Gain 11", "Gain 12", "Gain 21" )