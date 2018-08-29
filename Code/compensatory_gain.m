clear 
clc
%%  Filtering and playing an audio clip %%
%% Sound reader
NSampPerFrame = 2000;
audioInput2 = dsp.AudioFileReader('Filename', 'Audio/A_eng_m1.wav', 'SamplesPerFrame',NSampPerFrame);  
audioWriter2 = audioDeviceWriter('SampleRate',audioInput2.SampleRate);

Fs= 40000; % sampling frequency (Hz)
%Fs = audioInput2.SampleRate;
%% Audiogram 
frequencies = [250 500 1000 2000 4000 6000 8000];
values = [15 10 10 15 10 5 20];
%values = values;
semilogx(frequencies, values, '-o', 'MarkerSize', 25, 'LineWidth',4)
ylim([-30 5])
xlabel('Frequency (Hz)', 'FontSize',24)
ylabel('Air Threshold (dB)', 'FontSize',24)
set(gca,'fontsize',24)
grid on
% set(gcf, 'Position', [100 100 150 150]);
% saveas(gcf, 'test.png');
% export_fig test2.png
%%
requiredFrequencies = linspace(1, 8*10^3, 8*10^3);
vq = interp1(frequencies,values,requiredFrequencies);

audiogramdB = [vq(250), vq(315), vq(400), vq(500), vq(630), vq(800), ... 
vq(1000), vq(1250), vq(1600), vq(2000), vq(2500), vq(3150), ...   
vq(4000), vq(5000), vq(6300), vq(8000)];
audiogram=zeros(1,16);
for i=1: length(audiogramdB)
    audiogram(i) = db2mag(audiogramdB(i));
end


%% Filters
oneThirdOctaveFilterBank = createOneThirdOctaveFilters(14, Fs);
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
dRC = compressor('SampleRate', Fs);
SpecAna = dsp.SpectrumAnalyzer('PlotAsTwoSidedSpectrum',false, ...
    'SampleRate',40000, ...
    'NumInputPorts',2,...
    'ShowLegend',true, ...
    'YLimits',[-145,45]);

SpecAna.ChannelNames = {'Original signal','Amplified signal'};
counter = 1;
finalllllResult = zeros(1000,1);
figure
%while ~isDone(audioInput2)
for i=1:5
    
    buffer = audioInput2();  % Load a frame of audio

    output = filterOctave(oneThirdOctaveFilterBank,buffer, audiogram);
    output = dRC(output);
  
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
    
   % pause(0.01)
    plot(freqx2, magnitudeOriginal,'r');
    hold on
    plot(freqx, magnitudeFiltered, 'b');
    xlim([0 8000])
    drawnow
    hold off
   % audioWriter2(output);
    
    
    

%finalllllResult =  [finalllllResult; buffer];
            

end

%plot(finalllllResult);
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

%% 
    output = filterOctave(oneThirdOctaveFilterBank,buffer, audiogram);
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
    
    frequencies = [224 282 355 447 562 708 891 1120 1410 1780 2240 2820 3550 4470 5620 7080 8910];
    stuff = ones(length(frequencies))*-20;
   
    plot(freqx2, magnitudeOriginal,'r', 'LineWidth',2);
    hold on
    plot(freqx, magnitudeFiltered, 'b', 'LineWidth',2);
    hold on
    h =stem(frequencies, stuff, 'LineWidth',0.9);
    set(h,'BaseValue',-115);
    set(h, 'Marker', 'none')
    set(h, 'Color', [169/255,169/255,169/255]);
    xlim([0 8000])
    ylim([-115 -20])
    hold off
    xlabel('Frequency (Hz)', 'FontSize',16)
    ylabel('Magnitude (dB)', 'FontSize',16)
    set(gca,'fontsize',16)
    legend("Original signal", "Compensated signal")
    %%
 
%     transferFuncEstimator = dsp.TransferFunctionEstimator...
%                                  ('FrequencyRange','onesided',...
%                                   'SpectralAverages',20);
%     L = 2000;
% yw = zeros(L,16);
%     buffer2 = randn(L,1);
%     for i=1:16
%         oneThirdOctaveFilter = oneThirdOctaveFilterBank{i};
%         yw(:,i) = oneThirdOctaveFilter(buffer2);
%         %yw(:,i) = yw(:,i) * audiogram(i);
%     end
%    y = yw(:,1)+ yw(:,2)+ yw(:,3)+ yw(:,4) +yw(:,5)+ yw(:,6)+ yw(:,7)+ yw(:,8)+ yw(:,9)+ yw(:,10)+ yw(:,11)+ yw(:,12)+ yw(:,13)+ yw(:,14)+ yw(:,15)+ yw(:,16);
%    z  = transferFuncEstimator(repmat(buffer2,1,17),[ yw(:,1), yw(:,2), yw(:,3), yw(:,4) ,yw(:,5), yw(:,6), yw(:,7), yw(:,8), yw(:,9), yw(:,10), yw(:,11), yw(:,12), yw(:,13), yw(:,14), yw(:,15), yw(:,16), y]);
% 
%    freqNew = transpose(freqx2);
%    z = 20*log10(abs(z));
%    plot(freqNew, z)
%   
%    hold on
%    plot(freqx2, magnitudeOriginal,'r');
%    hold on
%    plot(freqx, magnitudeFiltered, 'b');
%    xlim([0 8000])
%    ylim([-115 5])
%    hold off
%     xlabel('Frequency (Hz)', 'FontSize',24)
%     ylabel('Magnitude (dB)', 'FontSize',24)
%     set(gca,'fontsize',24)
%     legend("Original signal", "Compensated signal")