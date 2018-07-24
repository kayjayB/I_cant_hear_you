%%  Filtering and playing an audio clip %%
Fs= 40000; % sampling frequency (Hz)

%% Audiogram 
frequencies = [250 500 1000 2000 4000 6000 8000];
values = [15 10 10 15 10 5 20];
requiredFrequencies = linspace(1, 8*10^3, 8*10^3);
vq = interp1(frequencies,values,requiredFrequencies);

audiogramdB = [vq(250), vq(315), vq(400), vq(500), vq(630), vq(800), ... 
vq(1000), vq(1250), vq(1600), vq(2000), vq(2500), vq(3150), ...   
vq(4000), vq(5000), vq(6300), vq(8000)];
audiogram=zeros(1,16);
for i=1: length(audiogramdB)
    audiogram(i) = db2mag(audiogramdB(i));
end

%% Sound reader
audioInput2 = dsp.AudioFileReader('Filename', 'Audio\B_eng_f1.wav');  
audioWriter2 = audioDeviceWriter('SampleRate',audioInput2.SampleRate);


%% Filters
oneThirdOctaveFilterBank = createOneThirdOctaveFilters(14);
%% Amplification

% To have a measure of reference, play the original audio once
%  while ~isDone(audioInput)
%      input = audioInput();  % Load a frame
%      audioWriter(input);  % Play the frame
%  end
% %Wait until audio is played to the end
% 
release(audioInput2);       % Close input file
release(audioWriter2);               % Close audio output device

SpecAna = dsp.SpectrumAnalyzer('PlotAsTwoSidedSpectrum',false, ...
    'SampleRate',40000, ...
    'NumInputPorts',2,...
    'ShowLegend',true, ...
    'YLimits',[-145,45]);

SpecAna.ChannelNames = {'Original signal','Amplified signal'};
counter = 1;

figure
while ~isDone(audioInput2)
    
    buffer = audioInput2();  % Load a frame of audio

    output = filterOctave(oneThirdOctaveFilterBank,buffer, audiogram);

    audioWriter2(output);
  
    n=length(output);   % create a scaling variable equal to the 
                            % length of the data
                    
    xFourier = fft(output)/n; % fourier transform of the data
    xFourier = 2*xFourier(1:length(output)/2+1); % create a one sided
                                                    % frequency spectrum                                     
    magnitudeFiltered = 20*log10(abs(xFourier));
    freqx = linspace(0,Fs/2,length(output)/2+1);
    
    nOriginal=length(buffer); 
    originalFourier = fft(buffer)/nOriginal;
    originalFourier = 2*originalFourier(1:length(buffer)/2+1); 
    magnitudeOriginal = 20*log10(abs(originalFourier));
    freqx2 = linspace(0,Fs/2,length(buffer)/2+1);
    %cla
            plot(freqx2, magnitudeOriginal,'r');
            hold on
            plot(freqx, magnitudeFiltered, 'b');
            xlim([0 8000])
            drawnow
            hold off
            

end


%% Loop with sine waves
% Sine1 = dsp.SineWave('SampleRate',samplingRate,'Frequency',250);
% % Sine2 = dsp.SineWave('Frequency',300,'SampleRate',samplingRate);
% % Sine3 = dsp.SineWave('Frequency',500,'SampleRate',samplingRate);
% 
% Sine1.SamplesPerFrame = 4000;
% %Sine2.SamplesPerFrame = 4000;
% %Sine3.SamplesPerFrame = 4000;
% 
% 
% for i = 1 : 2
%     x = Sine1();
%     output = filterFrames(x, audiogram);
% 
%     % FFT for the output
%     n=length(output);     
%     xFourier = fft(output-mean(output))/n; %
%     xFourier = 2*xFourier(1:length(output)/2+1);    
%     magnitudeFiltered = 20*log10(abs(xFourier)); % 
%     freqNew =linspace(0,Fs/2,length(output)/2+1); % 
%     
%     % FFT for the input
%     nOriginal=length(buffer); %
%     originalFourier = fft(buffer-mean(buffer))/nOriginal; %
%     originalFourier = 2*originalFourier(1:length(buffer)/2+1); %
%     magnitudeOriginal = 20*log10(abs(originalFourier)); %     
%     freqOld = linspace(0,Fs/2,length(buffer)/2+1); %     %cla % 
%     
%      plot(freqNew, magnitudeFiltered, 'b'); % 
%     drawnow % 
%     
%     
%     hold on 
%       plot(freqOld, magnitudeOriginal,'r'); %             
%     drawnow           
%     hold off
% end
% release(SpecAna)