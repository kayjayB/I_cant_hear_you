clc
clear all
if ~isempty(instrfind);
    fclose(instrfind);
    delete(instrfind)
end
close all
clc
disp('Serial Port Closed') 

Port = '/dev/cu.usbmodem1421' ;

[accelerometer.s, flag ]= SerialComs(Port) ;

%a = arduino('/dev/cu.usbmodem1421');

voltages=zeros(1,1000);

time=zeros(1,1000);

for i=1:1000
tic    
%voltages(i)=readVoltage(a, 'A0');
voltages(i) = AccelerometerInput(accelerometer); 
time(i)= toc;
end

plot(voltages);
mean(time)

%%