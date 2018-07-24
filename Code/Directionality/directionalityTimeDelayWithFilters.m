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
index2 =1;
weightTableTimeDelay = zeros(length(phaseDelay),n);

for r=1:length(theta)
    for microphone=n:-1:1
        fmid = F0(index2);
        weightTableTimeDelay(r,n+1-microphone)= weightTableAngles(r,n+1-microphone)/(fmid*2*pi);
    end
end

%deltaTperSample = 1/Fs;

%numberOfShifts = round(weightTableTimeDelay/deltaTperSample);
%% Array formation

microphone = phased.OmnidirectionalMicrophoneElement('FrequencyRange',[20 8e3]);
n = 4; %no of microphones
array = phased.ULA(n,d,'Element',microphone,'ArrayAxis','x');
c = 343; %speed of sound

%##########################################################################
%######Simulating sounds and noise in different directions#################

angleTone=[0;0];

fs=22050;
collector=phased.WidebandCollector('Sensor',array,'PropagationSpeed',c,...
    'SampleRate',fs,'NumSubbands',1000,'ModulatedInput',...
    false);

t_duration = 3;  % 3 seconds
t = 0:1/fs:t_duration-1/fs;

% preallocate
NSampPerFrame = 100000;
NTSample = t_duration*fs;
%%
% set up audio device writer
toneFileReader = dsp.AudioFileReader('SamplesPerFrame',NSampPerFrame);

audioWriter = audioDeviceWriter('SampleRate',toneFileReader.SampleRate, ...
        'SupportVariableSizeInput', true);
isAudioSupported = (length(getAudioDevices(audioWriter))>1);


simulatedAngle = 0; % (from dial)
correspondingRow = simulatedAngle/10 +1;
oneThirdOctaveFilterBank = createOneThirdOctaveFilters(14);
bandOutput = zeros(NSampPerFrame, n*length(F0));
result = zeros(NSampPerFrame, length(F0));

finalllllResult = zeros(NSampPerFrame,1);

while ~isDone(toneFileReader)
    x1 = toneFileReader();
    temp = collector([x1], [angleTone]); %+ ...

    index = 1;

    for i=0:4:length(F0)*n-n
        for j=1:4
            filterBand = oneThirdOctaveFilterBank{index};
            bandOutput(:,i+j) = filterBand(temp(:, j));
            filterBand.release();
            bandOutput(:,i+j) = delayseq(bandOutput(:,i+j),weightTableTimeDelay(correspondingRow, j),Fs);
        end
        index = index+1;
    end


    
    
    for i=1:NSampPerFrame
        for j=4:4:length(F0)*n
            result(i,j/4) = bandOutput(i,j-3)+bandOutput(i,j-2)+bandOutput(i,j-1)+bandOutput(i,j);

        end
    end

    playOutput = sum(result,2);
     audioWriter(playOutput/4);

    finalllllResult =  [finalllllResult; playOutput/4];
    
end

%%
audioWriter(finalllllResult);

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