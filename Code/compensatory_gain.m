%% Filtering a sin wave
samplingRate = 40*10^3;

Sine1 = dsp.SineWave('SampleRate',samplingRate,'Frequency',240);
Sine2 = dsp.SineWave('Frequency',300,'SampleRate',samplingRate);
Sine3 = dsp.SineWave('Frequency',500,'SampleRate',samplingRate);

filter24 = filterBand24;
filter25 = filterBand25;
filter26 = filterBand26;
filter27 = filterBand27;
filter28 = filterBand28;

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


%% Filtering and playing an audio clip

audioInput = dsp.AudioFileReader;  
audioWriter = audioDeviceWriter('SampleRate',audioInput.SampleRate);

% To have a measure of reference, play the original audio once
% while ~isDone(audioInput)
%     input = audioInput();  % Load a frame
%     audioWriter(input);  % Play the frame
% end
% Wait until audio is played to the end

pause(10*audioInput.SamplesPerFrame/audioInput.SampleRate);  
reset(audioInput);         % Reset to beginning of the file
% release(audioInput);       % Close input file
% release(audioWriter);               % Close audio output device

filter24 = filterBand24;
filter25 = filterBand25;
filter26 = filterBand26;
filter27 = filterBand27;
filter28 = filterBand28;

SpecAna = dsp.SpectrumAnalyzer('PlotAsTwoSidedSpectrum',false, ...
    'SampleRate',audioInput.SampleRate, ...
    'NumInputPorts',1,...
    'ShowLegend',true, ...
    'YLimits',[-145,45]);

while ~isDone(audioInput)
    input = audioInput();  % Load a frame of audio
    
     y = filter24(input);
     z = filter25(input);
     y = y*10; %How to multiply by a factor
     z = z*5;
     reconstructed = y + z;
     
     % Play the reconstructed audio frame
     audioWriter(reconstructed);
     SpecAna(input);
    
end

hello = 'done'