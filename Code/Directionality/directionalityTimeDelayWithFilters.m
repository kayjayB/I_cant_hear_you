clc
clear

%% create filter to get center frequencies %%
BW = '1/3 octave'; 
N = 14;
F0 = 1000;
Fs = 40000;
oneThirdOctaveFilter = octaveFilter('FilterOrder', N, ...
    'CenterFrequency', F0, 'Bandwidth', BW, 'SampleRate', Fs);
F0 = getANSICenterFrequencies(oneThirdOctaveFilter);
F0(F0<250) = [];
F0(F0>8000) = [];
%%
f = F0;
n = 10; % no of microphones
lambda = 343./f;

d=5*10^-2;
theta=0:(1/18)*pi:pi;

phaseDelay = zeros(16, length(theta));
for i=1:length(F0)
phaseDelay(i,:) = -(2*pi*(d/lambda(i)))*cos(theta);
end

weightTableAngles = zeros(length(phaseDelay),n*16); % 16 filters

% r=1;
index = 1;
for counter = 0:n:length(F0)*n-n
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
        weightTableTimeDelay(r,n+1-microphone)= weightTableAngles (r,n+1-microphone)/(fmid*2*pi);
    end
end

% deltaTperSample = 1/Fs;

% numberOfShifts = round(weightTableTimeDelay/deltaTperSample);
%% Array formation

microphone = phased.OmnidirectionalMicrophoneElement('FrequencyRange',[20 8e3],'BackBaffled',true);
array = phased.ULA(n,d,'Element',microphone,'ArrayAxis','x');
c = 343; %speed of sound
f = F0;
%figure;
%polarplot = plotResponse(array,f,c,'RespCut','Az','Format','Polar');

% ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ##
% ## ## ## Simulating sounds and noise in different directions ## ## ## ## ## ## ## ## #

angleTone=[80;0];

collector=phased.WidebandCollector('Sensor',array,'PropagationSpeed',c,...
    'SampleRate',Fs,'NumSubbands',50000,'ModulatedInput',...
    false);

t_duration = 3;  % 3 seconds
t = 0:1/Fs:t_duration-1/Fs;

% preallocate
NSampPerFrame = 10000;
NTSample = t_duration*Fs;
%%
% set up audio device writer
toneFileReader = dsp.AudioFileReader('SamplesPerFrame',NSampPerFrame);
%toneFileReader2 = dsp.AudioFileReader('Filename', 'Audio/B_eng_f1.wav','SamplesPerFrame',NSampPerFrame);
audioWriter = audioDeviceWriter('SampleRate',toneFileReader.SampleRate, ...
        'SupportVariableSizeInput', true);
isAudioSupported = (length(getAudioDevices(audioWriter))>1);

simulatedAngle = 0; % (from dial)
correspondingRow = simulatedAngle/10 +1;
oneThirdOctaveFilterBank = createOneThirdOctaveFilters(14, Fs);
bandOutput = zeros(NSampPerFrame, n*length(F0));
result = zeros(NSampPerFrame, length(F0));

angleTone2=[0;0];
while ~isDone(toneFileReader)
    x1 = toneFileReader();
    %x2 = toneFileReader2(); 
    temp = collector([x1], [angleTone]); %+ ...

    index = 1;

    for i=0:n:length(F0)*n-n
        for j=1:n
            filterBand = oneThirdOctaveFilterBank{index};
            bandOutput(:,i+j) = filterBand(temp(:, j));
            filterBand.release();
            bandOutput(:,i+j) = delayseq(bandOutput(:,i+j),weightTableTimeDelay(correspondingRow, j),Fs);
        end
        index = index+1;
    end

    for i=1:NSampPerFrame
        for j=n:n:length(F0)*n
            result(i,j/n) = bandOutput(i,j-9)+bandOutput(i,j-8)+bandOutput(i,j-7)+bandOutput(i,j-6)+bandOutput(i,j-5)+bandOutput(i,j-4)+bandOutput(i,j-3)+bandOutput(i,j-2)+bandOutput(i,j-1)+bandOutput(i,j);
        end
    end

    playOutput = sum(result,2);
    audioWriter(playOutput/n);
    

    
end

%%
% audioWriter(finalllllResult);

%%
band = 12; %choose the frequency band
simulatedAngle = 90; % (from dial)
angle=0:1:180;

weights=zeros(length(theta),n);
for i=band*n+1:band*n+n
    for j=1:19
        weights(j,i-band*n)=cos(weightTableAngles(j,i))+1i*sin(weightTableAngles(j,i));
    end
end
weights=conj(weights);
weights=transpose(weights);

steervec = phased.SteeringVector('SensorArray',array,'PropagationSpeed',c);

sv=steervec(f(band),simulatedAngle);

Ddata= directivity(array,f(band),angle,'PropagationSpeed',c,'Weights',sv);

guiData = zeros(length(angle),length(theta));
for i=1:length(theta)
    guiSteerVector=steervec(f(band),rad2deg(theta(i)));
    temp=directivity(array,f(band),angle,'PropagationSpeed',c,'Weights',guiSteerVector);
    for j=1:1:length(angle)
        guiData(j,i)=temp(j);
    end
end

guiDataMag=db2mag(guiData);

normalised = zeros(length(angle),length(theta));
maximum=max(guiDataMag);
minimum=min(guiDataMag);
for i=1:length(theta)
    for j=1:1:length(angle)
        %normalised(j,i)=(guiData(j,i)-minimum(i))/(maximum(i)-minimum(i));
        normalised(j,i)=(guiDataMag(j,i))/(maximum(i));
    end
end

guiDataX=zeros(length(angle),length(theta));
guiDataY=zeros(length(angle),length(theta));

for i=1:length(theta)
    for j=1:1:length(angle)
        guiDataX(j,i)= normalised(j,i)*cos(deg2rad(angle(j)));
    end
end

for i=1:length(theta)
    for j=1:1:length(angle)
        guiDataY(j,i)= normalised(j,i)*sin(deg2rad(angle(j)));
    end
end

column=(simulatedAngle/10)+1;
figure
plot(guiDataX(:,column),guiDataY(:,column))
xlim([-1 1]);
xticks([-1 -0.8 -0.6 -0.4 -0.2 0 0.2 0.4 0.6 0.8 1]);
xticklabels({'1.0','0.8','0.6','0.4','0.2','0','0.2','0.4','0.6','0.8','1.0'})
ylim([0 1]);
line([0 0],[0 1],'Color','Black')
line
%line([0 0],[-1 1],'Color','Black')

%line([0 0],[1 1],'Color','Black')
%grid on


% release(toneFileReader);       % Close input file
% release(audioWriter);  
% x1 = toneFileReader();
%     temp = collector([x1],...
%         [angleTone]); %+ ...
% oneThirdOctaveFilterBank = createOneThirdOctaveFilters(14);
% filterBand = oneThirdOctaveFilterBank{1};
% band1Mic1 = filterBand(temp(:, 1));
% filterBand.release();
% % oneThirdOctaveFilterBank = createOneThirdOctaveFilters(14);
% filterBand = oneThirdOctaveFilterBank{1};
% band1Mic2 = filterBand(temp(:, 1));
% % plot(temp(:,1))
% % hold on
% plot(band1Mic1);
% hold on 
% plot(band1Mic2);
%%
% release(toneFileReader);       % Close input file
% release(audioWriter);  
% 
% blah =toneFileReader();
% tempOut = sum(temp,2);
% audioWriter(temp(:,4));
recObj = audiorecorder;
%%

recObj = audiorecorder(44100, 16, 2);
disp('Start speaking.')
recordblocking(recObj, 10);
disp('End of Recording.');
%%
y=getaudiodata(recObj);

filename = 'kelvinVoice.wav';
audiowrite(filename,y,44100);


