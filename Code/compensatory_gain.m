%%  Filtering and playing an audio clip %%
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

%% Filters
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
%% Sound reader (for now)
audioInput = dsp.AudioFileReader('Filename', 'B_eng_f2.wav');  
audioWriter = audioDeviceWriter('SampleRate',audioInput.SampleRate);

%% Amplification
% To have a measure of reference, play the original audio once
 while ~isDone(audioInput)
     input = audioInput();  % Load a frame
     audioWriter(input);  % Play the frame
 end
%Wait until audio is played to the end

release(audioInput);       % Close input file
release(audioWriter);               % Close audio output device

SpecAna = dsp.SpectrumAnalyzer('PlotAsTwoSidedSpectrum',false, ...
    'SampleRate',audioInput.SampleRate, ...
    'NumInputPorts',2,...
    'ShowLegend',true, ...
    'YLimits',[-145,45]);

SpecAna.ChannelNames = {'Original signal','Amplified signal'};
counter = 1;

while ~isDone(audioInput)
    input = audioInput();  % Load a frame of audio
    tic
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
     endOfFiltering(counter) = toc ;
     % Play the reconstructed audio frame
     audioWriter(reconstructed);
     %SpecAna(input, reconstructed);
     counter = counter+1;
    
end

mean(endOfFiltering) % to find the average time of all the loops
