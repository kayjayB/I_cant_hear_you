clc
clear

%% create filter to get center frequencies %%
BW = '1/3 octave'; 
N = 14;
F0 = 1000;
Fs = 22050;
oneThirdOctaveFilter = octaveFilter('FilterOrder', N, ...
    'CenterFrequency', F0, 'Bandwidth', BW, 'SampleRate', Fs);
F0 = getANSICenterFrequencies(oneThirdOctaveFilter);
F0(F0<250) = [];
F0(F0>8000) = [];
%%
f = F0;
n = 4; %no of microphones
lambda = 343./f;

d=5*10^-2;
theta=0:(1/18)*pi:pi;

phaseDelay = zeros(16, length(theta));
for i=1:length(F0)
phaseDelay(i,:) = -(2*pi*(d/lambda(i)))*cos(theta);
end

weightTableAngles = zeros(length(phaseDelay),n*16); % 16 filters

%r=1;
index = 1;
for counter = 0:4:length(F0)*n-n
    for r=1:length(theta)
        for microphone=n:-1:1
           weightTableAngles(r,n+1-microphone+counter)= phaseDelay(index, r)*(microphone-1);
        end  
    end
    index = index +1;
end
%%
%convert weight table in terms of imaginary numbers
A=1;
weightTableImag = zeros(length(phaseDelay),n*16);
index = 1;
for counter = 0:4:length(F0)*n-n
    for r=1:length(theta)
        for microphone=1:n
            weightTableImag(r,microphone+counter)= A*(cos(weightTableAngles(r,microphone+counter))+1i*sin(weightTableAngles(r,microphone+counter)));
        end
    end
end
weightTableImag= conj(weightTableImag);
%%
%Array formation

microphone = phased.OmnidirectionalMicrophoneElement('FrequencyRange',[20 8e3]);
n = 4; %no of microphones
array = phased.ULA(n,d,'Element',microphone,'ArrayAxis','x');
c = 343; %speed of sound

pos = getElementPosition(array);

%figure;
%viewArray(array,'ShowIndex','all','ShowTaper',true);

figure;
polarplot = plotResponse(array,f(4),c,'RespCut','Az','Format','Polar');
% directivity = get(polarplot,'XData');
% angle=get(polarplot,'YData');
% 
% figure
% polarplot(angle,directivity)
angle=-180:1:180;
Ddata= directivity(array,f,angle,'PropagationSpeed',c);

%figure
%plot(angle,Ddata);

D=zeros(1,181);
for i=1:181
    D(i)=Ddata(i+180);
end

angleRad=linspace(0,pi,181);

figure
%polar(angleRad,db2mag(D))
halfPolar(angleRad,D);

%##########################################################################
%######Simulating sounds and noise in different directions#################

samplingFreq = 8000;
duration = 3; %the file properties is showing duration of 5s
t=[0: 1/samplingFreq: duration];
y=sin(2*pi*2000*t)';
audiowrite('3kHzSine.wav',y,samplingFreq);
[y,samplingFreq]=audioread('3kHzSine.wav');

angleTone=[90;0];

fs=22050;
collector=phased.WidebandCollector('Sensor',array,'PropagationSpeed',c,...
    'SampleRate',fs,'NumSubbands',1000,'ModulatedInput',...
    false);

t_duration = 3;  % 3 seconds
t = 0:1/fs:t_duration-1/fs;

prevS = rng(2008); %to represent thermal noise of each microphone
noisePwr = 1e-4; % noise power

% preallocate
NSampPerFrame = 100000;
NTSample = t_duration*fs;
%%
% set up audio device writer
toneFileReader = dsp.AudioFileReader('SamplesPerFrame',NSampPerFrame);

audioWriter = audioDeviceWriter('SampleRate',toneFileReader.SampleRate, ...
        'SupportVariableSizeInput', true);
isAudioSupported = (length(getAudioDevices(audioWriter))>1);


simulatedAngle = 90; % (from dial)
correspondingRow = simulatedAngle/10 +1;
oneThirdOctaveFilterBank = createOneThirdOctaveFilters(14);
bandOutput = zeros(NSampPerFrame, n*length(F0));
result = zeros(NSampPerFrame, length(F0));

finalllllResult = zeros(NSampPerFrame,1);
%simulate
%playOutput=zeros(length(tone),1);
while ~isDone(toneFileReader)
    x1 = toneFileReader();
    temp = collector([x1],...
        [angleTone]); %+ ...

    index = 1;

    for i=0:4:length(F0)*n-n
        for j=1:4
            filterBand = oneThirdOctaveFilterBank{index};
            bandOutput(:,i+j) = filterBand(temp(:, j));
            filterBand.release();
        end
        index = index+1;
    end
%     audioWriter(bandOutput(:,1)+bandOutput(:,5)+bandOutput(:,9)+bandOutput(:,13)+bandOutput(:,17) +...
%         bandOutput(:,21)+bandOutput(:,25)+bandOutput(:,29)+bandOutput(:,33)+bandOutput(:,37)+ ...
%        bandOutput(:,41)+bandOutput(:,45)+bandOutput(:,49)+bandOutput(:,53)+bandOutput(:,57)+ ...
%        bandOutput(:,61)+ bandOutput(:,2)+bandOutput(:,6)+bandOutput(:,10)+bandOutput(:,14)+bandOutput(:,18) +...
%         bandOutput(:,22)+bandOutput(:,26)+bandOutput(:,30)+bandOutput(:,34)+bandOutput(:,38)+ ...
%        bandOutput(:,42)+bandOutput(:,46)+bandOutput(:,50)+bandOutput(:,54)+bandOutput(:,58)+ ...
%        bandOutput(:,62));
   
%       audioWriter(bandOutput(:,3)+bandOutput(:,7)+bandOutput(:,11)+bandOutput(:,15)+bandOutput(:,19) +...
%         bandOutput(:,23)+bandOutput(:,27)+bandOutput(:,31)+bandOutput(:,35)+bandOutput(:,39)+ ...
%        bandOutput(:,43)+bandOutput(:,47)+bandOutput(:,51)+bandOutput(:,55)+bandOutput(:,59)+ ...
%        bandOutput(:,63));

%    variable = bandOutput(:,4)+bandOutput(:,8)+bandOutput(:,12)+bandOutput(:,16)+bandOutput(:,20) +...
%         bandOutput(:,24)+bandOutput(:,28)+bandOutput(:,32)+bandOutput(:,36)+bandOutput(:,40)+ ...
%        bandOutput(:,44)+bandOutput(:,48)+bandOutput(:,52)+bandOutput(:,56)+bandOutput(:,60)+ ...
%        bandOutput(:,64);
%      audioWriter(variable);
 
    for i=1:NSampPerFrame
        for j=4:4:length(F0)*n
            result(i,j/4) = bandOutput(i,j-3)+bandOutput(i,j-2)+bandOutput(i,j-1)+bandOutput(i,j);
          %  result(i,j/4) = abs(result(i,j/4));
        end
    end

    playOutput = sum(result,2);
     audioWriter(playOutput/4);

    finalllllResult =  [finalllllResult; playOutput/4];
    
end

%%
audioWriter(finalllllResult);
% 
% ang_dft = [-30; 0]; %[azimuthAng; elevationAng]
% ang_cleanspeech = [-10; 10];
% ang_laughter = [90; 0];
% 
% fs = 8000;
% collector = phased.WidebandCollector('Sensor',array,'PropagationSpeed',c,...
%     'SampleRate',fs,'NumSubbands',1000,'ModulatedInput', false);
% 
% t_duration = 3;  % 3 seconds
% t = 0:1/fs:t_duration-1/fs;
% 
% prevS = rng(2008); %to represent thermal noise of each microphone
% noisePwr = 1e-4; % noise power
% 
% % preallocate
% NSampPerFrame = 1000;
% NTSample = t_duration*fs;
% sigArray = zeros(NTSample,n);
% voice_dft = zeros(NTSample,1);
% voice_cleanspeech = zeros(NTSample,1);
% voice_laugh = zeros(NTSample,1);
% 
% % set up audio device writer
% audioWriter = audioDeviceWriter('SampleRate',fs, ...
%         'SupportVariableSizeInput', true);
% isAudioSupported = (length(getAudioDevices(audioWriter))>1);
% 
% dftFileReader = dsp.AudioFileReader('dft_voice_8kHz.wav',...
%     'SamplesPerFrame',NSampPerFrame);
% speechFileReader = dsp.AudioFileReader('cleanspeech_voice_8kHz.wav',...
%     'SamplesPerFrame',NSampPerFrame);
% laughterFileReader = dsp.AudioFileReader('laughter_8kHz.wav',...
%     'SamplesPerFrame',NSampPerFrame);
% 
% % simulate
% for m = 1:NSampPerFrame:NTSample
%     sig_idx = m:m+NSampPerFrame-1;
%     x1 = dftFileReader();
%     x2 = speechFileReader();
%     x3 = 2*laughterFileReader();
%     temp = collector([x1 x2 x3],...
%         [ang_dft ang_cleanspeech ang_laughter]) + ...
%         sqrt(noisePwr)*randn(NSampPerFrame,n);
%     if isAudioSupported
%         play(audioWriter,0.5*temp(:,3));
%     end
%     sigArray(sig_idx,:) = temp;
%     voice_dft(sig_idx) = x1;
%     voice_cleanspeech(sig_idx) = x2;
%     voice_laugh(sig_idx) = x3;
% end

%%
release(toneFileReader);       % Close input file
release(audioWriter);  
x1 = toneFileReader();
    temp = collector([x1],...
        [angleTone]); %+ ...
oneThirdOctaveFilterBank = createOneThirdOctaveFilters(14);
filterBand = oneThirdOctaveFilterBank{1};
band1Mic1 = filterBand(temp(:, 1));
filterBand.release();
%oneThirdOctaveFilterBank = createOneThirdOctaveFilters(14);
filterBand = oneThirdOctaveFilterBank{1};
band1Mic2 = filterBand(temp(:, 1));
% plot(temp(:,1))
% hold on
plot(band1Mic1);
hold on 
plot(band1Mic2);