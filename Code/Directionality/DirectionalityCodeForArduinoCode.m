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
n = 4; % no of microphones
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
NSampPerFrame = 50;
NTSample = t_duration*Fs;
%%
% set up audio device writer
toneFileReader = dsp.AudioFileReader('SamplesPerFrame',NSampPerFrame);
toneFileReader2 = dsp.AudioFileReader('SamplesPerFrame',NSampPerFrame);
audioWriter = audioDeviceWriter('SampleRate',toneFileReader.SampleRate, ...
        'SupportVariableSizeInput', true);
isAudioSupported = (length(getAudioDevices(audioWriter))>1);

simulatedAngle = 90; % (from dial)
correspondingRow = simulatedAngle/10 +1;
oneThirdOctaveFilterBank = createOneThirdOctaveFilters(14, Fs);
bandOutput = zeros(NSampPerFrame, n*length(F0));
result = zeros(NSampPerFrame, length(F0));

angleTone2=[0;0];
while ~isDone(toneFileReader)
    x1 = toneFileReader();
    x2 = toneFileReader2(); 
    temp = collector([x1], [angleTone]); %+ ...

    %i=0:n:length(F0)*n-n;
    j=1:n;
    
    for i=0:n:length(F0)*n-n
        filterBand = oneThirdOctaveFilterBank{i/n+1};
        bandOutput(:,i+j) = filterBand(temp(:, j));
        filterBand.release();
       % bandOutput(:,i+j) = bandOutput(:,i+j)*audiogram(i/n+1);
        %bandOutput(:,i+j) = delayseq(bandOutput(:,i+j),weightTableTimeDelay(correspondingRow, j),Fs);
        bandOutput(:,i+j)  = timeDelay(bandOutput(:,i+j),weightTableTimeDelay(correspondingRow, j), Fs);
    end
    
    i=1:1:NSampPerFrame; 
    j=n:n:length(F0)*n;
    %result(i,j/n) = bandOutput(i,j-9)+bandOutput(i,j-8)+bandOutput(i,j-7)+bandOutput(i,j-6)+bandOutput(i,j-5)+bandOutput(i,j-4)+bandOutput(i,j-3)+bandOutput(i,j-2)+bandOutput(i,j-1)+bandOutput(i,j);
    result(i,j/n) = bandOutput(i,j-3)+bandOutput(i,j-2)+bandOutput(i,j-1)+bandOutput(i,j);



    playOutput = sum(result,2);
    audioWriter(playOutput/n);
        
end

%%
% audioWriter(finalllllResult);

%%
band = 12; %choose the frequency band
simulatedAngle = 140; % (from dial)
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

figure
plot(angle,Ddata)


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

result = 1:1:200;
  for idx1 = 1:4
    for idx0 = 1:50 
      result(idx0 + 50 * idx1)
    end
  end
%%  
    for idx0 = 1:4:50*4 
      result(idx0)
    end