clc
clear
% f = 4000;
% c = 343;
% d = (c/4000)/2;
% %d=12*10^-3;
% tau = 0.5*d/c;
% theta = 0:0.01:2*pi;
% t0 = tau + (d/c)*cos(theta);
% phase = pi/2 -((0.5*2*pi*f)*t0);
% B = sqrt(2-2*cos(2*pi*f*t0));
% %H = B.*exp(1i*phase);
% 
% figure
% polarplot(theta,B)

f = 3430;
n = 4; %no of microphones
lambda = 343/f;
%d=lambda/2;
d=5*10^-2;
theta=0:(1/18)*pi:pi;
phaseDelay = -(2*pi*(d/lambda))*cos(theta);

weightTableAngles = zeros(length(phaseDelay),n);

%r=1;
for r=1:length(theta)
    index=0;
    for microphone=n:-1:1
       weightTableAngles(r,microphone)= phaseDelay(r)*(n-4+index);
       index=index+1;
    end
end

weightTableAngles = conj(weightTableAngles);

%convert weight table in terms of imaginary numbers
A=1;
weightTableImag = zeros(length(phaseDelay),n);
for r=1:length(theta)
    for microphone=1:n
        weightTableImag(r,microphone)= A*(cos(weightTableAngles(r,microphone))+1i*sin(weightTableAngles(r,microphone)));
    end
end

%%
%Array formation

taper = weightTableImag(5,:);
microphone = phased.OmnidirectionalMicrophoneElement('FrequencyRange',[20 20e3]);
n = 4; %no of microphones
array = phased.ULA(n,d,'Element',microphone,'ArrayAxis','x','Taper',conj(taper));
c = 343; %speed of sound

pos = getElementPosition(array);

figure;
viewArray(array);

figure;
polarplot = plotResponse(array,f,c,'RespCut','Az','Format','Polar');
% directivity = get(polarplot,'XData');
% angle=get(polarplot,'YData');
% 
% figure
% polarplot(angle,directivity)
angle=-180:1:180;
Ddata= directivity(array,f,angle,'PropagationSpeed',c);

figure
plot(angle,Ddata);

D=zeros(1,181);
for i=1:181
    D(i)=Ddata(i+180);
end

angleRad=linspace(0,pi,181);

figure
%polar(angleRad,db2mag(D))
halfPolar(angleRad,D)







