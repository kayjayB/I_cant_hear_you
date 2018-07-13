%% Filtering a sin wave
samplingRate = 40*10^3;

Sine1 = dsp.SineWave('SampleRate',samplingRate,'Frequency',240);
Sine2 = dsp.SineWave('Frequency',300,'SampleRate',samplingRate);
Sine3 = dsp.SineWave('Frequency',500,'SampleRate',samplingRate);

filter24 = filterBand24;
filter25 = filterBand25;

%fvtool(filter24, 1, filter25, 1, filter26, 1, filter27, 1, filter28);

SpecAna = dsp.SpectrumAnalyzer('PlotAsTwoSidedSpectrum',false, ...
    'SampleRate',Sine1.SampleRate, ...
    'NumInputPorts',3,...
    'ShowLegend',true, ...
    'YLimits',[-145,45]);

SpecAna.ChannelNames = {'Original noisy signal','Low pass filtered signal'};

Sine1.SamplesPerFrame = 4000;
Sine2.SamplesPerFrame = 4000;
Sine3.SamplesPerFrame = 4000;

scope = dsp.TimeScope('NumInputPorts',3, 'SampleRate',Sine1.SampleRate,...
    'TimeSpan',0.1, 'ShowLegend', true);

for i = 1 : 100
    x = Sine1()+Sine2()+Sine3()+0.1.*randn(Sine1.SamplesPerFrame,1);
    y = filter24(x);
    %y = filtfilt(filter24.Numerator,1,x);
    %grpdelay(filter24,N,Fs)
    z = filter25(x);
    %z = filtfilt(filter25.Numerator,1,x);
    % y = y*2; How to multiply by a factor
    a = y + z;
    %scope(y, z, a);
    SpecAna(y, z,a);
    
end
release(SpecAna)

%% Audiogram 
frequencies = [250 500 1000 2000 4000 6000 8000];
values = [15 10 10 15 10 5 20];
requiredFrequencies = linspace(1, 8*10^3, 8*10^3);
vq = interp1(frequencies,values,requiredFrequencies);
figure(1)
plot(requiredFrequencies, vq);
audiogramdB = [vq(250), vq(315), vq(400), vq(500), vq(630), vq(800), ... 
vq(1000), vq(1250), vq(1600), vq(2000), vq(2500), vq(3150), ...   
vq(4000), vq(5000), vq(6300), vq(8000)];
audiogram=zeros(1,16);
for i=1: length(audiogramdB)
    audiogram(i) = db2mag(audiogramdB(i));
end

%% Filtering and playing an audio clip

audioInput = dsp.AudioFileReader;  
audioWriter = audioDeviceWriter('SampleRate',audioInput.SampleRate);

% To have a measure of reference, play the original audio once
 while ~isDone(audioInput)
     input = audioInput();  % Load a frame
     audioWriter(input);  % Play the frame
 end
% Wait until audio is played to the end

% release(audioInput);       % Close input file
% release(audioWriter);               % Close audio output device

filter24 = filterBand24;
filter25 = filterBand25;
filter26 = filterBand26;
filter27 = filterBand27;
filter28 = filterBand28;
filter29 = filterBand29;
filter30 = filterBand30;
filter31 = filterBand31;
filter32 = filterBand32;
filter33 = filterBand33;
filter34 = filterBand34;
filter35 = filterBand35;
filter36 = filterBand36;
filter37 = filterBand37;
filter38 = filterBand38;
filter39 = filterBand39;

SpecAna = dsp.SpectrumAnalyzer('PlotAsTwoSidedSpectrum',false, ...
    'SampleRate',audioInput.SampleRate, ...
    'NumInputPorts',1,...
    'ShowLegend',true, ...
    'YLimits',[-145,45]);

while ~isDone(audioInput)
    input = audioInput();  % Load a frame of audio
    
     a = filter24(input);
     a = a * audiogram(1);
     b = filter25(input);
     b = b * audiogram(2);
     c = filter26(input);
     c = c * audiogram(3);
     d = filter27(input);
     d = d * audiogram(4);
     e = filter28(input);
     e = e * audiogram(5);
     f = filter29(input);
     f = f * audiogram(6);
     g = filter30(input);
     g = g * audiogram(7);
     h = filter31(input);
     h = h * audiogram(8);
     i = filter32(input);
     i = i * audiogram(9);
     j = filter33(input);
     j = j * audiogram(10);
     k = filter34(input);
     k = k * audiogram(11);
     l = filter35(input);
     l = l * audiogram(12);
     m = filter36(input);
     m = m * audiogram(13);
     n = filter37(input);
     n = n * audiogram(14);
     o = filter38(input);
     o = o * audiogram(15);
     p = filter39(input);
     p = p * audiogram(16);

     reconstructed = a+ b+ c+ d +e+ f+ g+ h+ i+ j+ k+ l+ m+ n+ o+ p;
     
     % Play the reconstructed audio frame
     audioWriter(reconstructed);
     SpecAna(input);
    
end