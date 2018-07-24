microphone = phased.OmnidirectionalMicrophoneElement('FrequencyRange',[20 20e3]);
n = 4; %no of microphones
d = 0.05;
array = phased.ULA(n,d,'Element',microphone,'ArrayAxis','x');
c = 343; %speed of sound

ang_dft = [80; 0];
ang_cleanspeech = [90; 0];
ang_laughter = [180; 0];

fs = 8000;
collector = phased.WidebandCollector('Sensor',array,'PropagationSpeed',c,...
    'SampleRate',fs,'NumSubbands',1000,'ModulatedInput', false);

t_duration = 6;  % 3 seconds
t = 0:1/fs:t_duration-1/fs;

% preallocate
NSampPerFrame = 1000;
NTSample = t_duration*fs;
sigArray = zeros(NTSample,n);
voice_dft = zeros(NTSample,1);
voice_cleanspeech = zeros(NTSample,1);
voice_laugh = zeros(NTSample,1);

% set up audio device writer
player = audioDeviceWriter('SampleRate',fs);

dftFileReader = dsp.AudioFileReader('SpeechDFT-16-8-mono-5secs.wav',...
    'SamplesPerFrame',NSampPerFrame);
speechFileReader = dsp.AudioFileReader('FemaleSpeech-16-8-mono-3secs.wav',...
    'SamplesPerFrame',NSampPerFrame);
laughterFileReader = dsp.AudioFileReader('Laughter-16-8-mono-4secs.wav',...
    'SamplesPerFrame',NSampPerFrame);

% simulate
for m = 1:NSampPerFrame:NTSample
    sig_idx = m:m+NSampPerFrame-1;
    x1 = dftFileReader();
    x2 = speechFileReader();
    x3 = 0*laughterFileReader();
    temp = collector([x1 x2 x3],[ang_dft ang_cleanspeech ang_laughter]);
    %player(0.5*temp(:,3));
    sigArray(sig_idx,:) = temp;
    voice_dft(sig_idx) = x1;
    voice_cleanspeech(sig_idx) = x2;
    voice_laugh(sig_idx) = x3;
end

angSteer = [60;0]; %steering beam in this direction
beamformer = phased.TimeDelayBeamformer('SensorArray',array,...
    'SampleRate',fs,'Direction',angSteer,'PropagationSpeed',c,'WeightsOutputPort',true);

signalsource = dsp.SignalSource('Signal',sigArray,...
    'SamplesPerFrame',NSampPerFrame);

cbfOut = zeros(NTSample,1);

for m = 1:NSampPerFrame:NTSample
    temp = beamformer(signalsource());
    player(temp);
    cbfOut(m:m+NSampPerFrame-1,:) = temp;
end


%[Y,Weights]=step(beamformer,signalsource());

freq = [1000 2750];
fc = 2000;

plotFreq = linspace(min(freq),max(freq),1);
pattern(microphone,plotFreq,[0:180],0,'CoordinateSystem','polar')

pattern(array,1000,[0:180],0,'CoordinateSystem','polar',...
    'PropagationSpeed',c)

% [y,w,centerfreqs] = beamformer(signalsource());

% pattern(array,centerfreqs.',[0:180],0,'Weights',w,'CoordinateSystem','polar',...
%     'PropagationSpeed',c)



