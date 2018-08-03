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
weightTableTimeDelayTemp = zeros(length(phaseDelay),n);

for r=1:length(theta)
    for microphone=n:-1:1
        fmid = F0(index2);
        weightTableTimeDelayTemp(r,n+1-microphone)= weightTableAngles(r,n+1-microphone)/(fmid*2*pi);
    end
end
weightTableTimeDelay= zeros(length(phaseDelay),8);
weightTableTimeDelay(:,1) = weightTableTimeDelayTemp(:,1);
weightTableTimeDelay(:,3) = weightTableTimeDelayTemp(:,2);
weightTableTimeDelay(:,5) = weightTableTimeDelayTemp(:,3);
weightTableTimeDelay(:,7) = weightTableTimeDelayTemp(:,4);
weightTableTimeDelay(:,2) = weightTableTimeDelayTemp(:,1);
weightTableTimeDelay(:,4) = weightTableTimeDelayTemp(:,2);
weightTableTimeDelay(:,6) = weightTableTimeDelayTemp(:,3);
weightTableTimeDelay(:,8) = weightTableTimeDelayTemp(:,4);
% deltaTperSample = 1/Fs;

% numberOfShifts = round(weightTableTimeDelay/deltaTperSample);
%% Array formation
n=8;
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
audioWriter = audioDeviceWriter('SampleRate',toneFileReader.SampleRate, ...
        'SupportVariableSizeInput', true);
isAudioSupported = (length(getAudioDevices(audioWriter))>1);

simulatedAngle = 90; % (from dial)
correspondingRow = simulatedAngle/10 +1;
oneThirdOctaveFilterBank = createOneThirdOctaveFilters(14, Fs);
bandOutput = zeros(NSampPerFrame, n*length(F0));
result = zeros(NSampPerFrame, length(F0));

angleTone2=[0;0];
%while ~isDone(toneFileReader)
    x1 = toneFileReader();
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
    result(i,j/n) = bandOutput(i,j-7)+bandOutput(i,j-6)+bandOutput(i,j-5)+bandOutput(i,j-4)+bandOutput(i,j-3)+bandOutput(i,j-2)+bandOutput(i,j-1)+bandOutput(i,j);
    %result(i,j/n) = bandOutput(i,j-3)+bandOutput(i,j-2)+bandOutput(i,j-1)+bandOutput(i,j);



    playOutput = sum(result,2);
    audioWriter(playOutput/n);
        
%end


% result = 1:1:200;
%   for idx1 = 1:50
%     for idx0 = 1:8 
%       result(idx0 + 8 * idx1)
%     end
%   end
% %%  
%     for idx0 = 1:4:50*4 
%       result(idx0)
%     end

%%

