clc
clear all
if ~isempty(instrfind);
    fclose(instrfind);
    delete(instrfind)
end
close all
clc
disp('Serial Port Closed') 

Port = 'COM3' ;

[accelerometer.s, flag ]= SerialComs(Port) ;

%a = arduino('/dev/cu.usbmodem1421');

voltages=zeros(1,1000);

time=zeros(1,1000);

tic
for i=1:1000    
%voltages(i)=readVoltage(a, 'A0');
voltages(i) = AccelerometerInput(accelerometer); 
end
time = toc;
samplingFreq=1000/time;

plot(voltages);
%mean(time)
