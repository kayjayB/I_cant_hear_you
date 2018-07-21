% clc
% clear
% if ~isempty(instrfind)
%     fclose(instrfind);
%     delete(instrfind)
% end
% close all
% clc
% disp('Serial Port Closed') 
% 
% Port = '/dev/cu.usbmodem1421' ;
% 
% [accelerometer.s, flag ]= SerialComs(Port) ;
% 
% %a = arduino('/dev/cu.usbmodem1421');
% 
% voltages=zeros(1,10000);
% 
% time=zeros(1,100*100);
% 
% time1 = tic;
% fprintf("here")
% % for i=1:100*100
% % time2 = tic  ;  
% % %voltages(i)=readVoltage(a, 'A0');
% % voltages(i) = AccelerometerInput(accelerometer); 
% % time(i)= toc(time2);
% % end
% 
% time100Samples = AccelerometerInput(accelerometer)
% 
% totalTime=toc(time1);
% 
% plot(voltages);
% mean(time)

%% Code to use filters
%
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
%

fs = 40000;  
endOfRange = 20*10^3;
t = 0 : 1/fs : 2; 
f1 = 280; 
f2 = 420;
f3 = 8000;
A = 1;           
y1 = A * sin( 2 * pi * f1 * t);
y2 = A * sin( 2 * pi * f2 * t);
y3 = A * sin( 2 * pi * f3 * t);
input =  y3;
frame = zeros(1, 2014);
counter=1;

for i=1:1024:length(input)-1024
    buffer = input(i:i+1024);
%     disp(['i is: ', num2str(i)]);
%     disp(['i+1024 is: ', num2str(i+1024)]);
    output = filterFrames(buffer, audiogram);
end
plot(input)
hold on
plot(output)
hold off
