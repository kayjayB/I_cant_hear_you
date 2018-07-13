samplingRate = 40*10^3;

Sine1 = dsp.SineWave('SampleRate',samplingRate,'Frequency',240);
Sine2 = dsp.SineWave('Frequency',300,'SampleRate',samplingRate);
Sine3 = dsp.SineWave('Frequency',500,'SampleRate',samplingRate);
% 
% filter24 = dsp.LowpassFilter('PassbandFrequency',5000,...
%     'StopbandFrequency',8000, 'StopbandAttenuation', 60, ...
%     'PassbandRipple', 1);

filter24 = filterBand24;
filter25 = filterBand25;

cascadedFilt = dsp.FilterCascade(filter24,filter25); % Cascading the two filters
% fvtool(filter24);
% fvtool(filter25);

fvtool(cascadedFilt);
%%

%fvtool(filter24,'Analysis','freq')

SpecAna = dsp.SpectrumAnalyzer('PlotAsTwoSidedSpectrum',false, ...
    'SampleRate',Sine1.SampleRate, ...
    'NumInputPorts',2,...
    'ShowLegend',true, ...
    'YLimits',[-145,45]);

SpecAna.ChannelNames = {'Original noisy signal','Low pass filtered signal'};

Sine1.SamplesPerFrame = 4000;
Sine2.SamplesPerFrame = 4000;
Sine3.SamplesPerFrame = 4000;


for i = 1 : 500
    x = Sine1()+Sine2()+Sine3()+0.1.*randn(Sine1.SamplesPerFrame,1);
    %y = filter24(x); 
    y= cascadedFilt(x);
    SpecAna(x,y);                              
end
release(SpecAna)