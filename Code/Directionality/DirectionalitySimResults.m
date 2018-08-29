clc
clear
f = 3340;
n = 10; %no of microphones
lambda = 343/f;
%d=lambda/2;
d=5*10^-2;
theta=0:(1/18)*pi:pi;
phaseDelay = -(2*pi*(d/lambda))*cos(theta);

weightTableAngles = zeros(length(phaseDelay),n);

%r=1;
for r=1:length(theta)
    for microphone=n:-1:1
       weightTableAngles(r,n+1-microphone)= phaseDelay(r)*(microphone-1);
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

microphone = phased.OmnidirectionalMicrophoneElement('FrequencyRange',[20 20e3],'BackBaffled',true);

%% 90 degrees
taper = weightTableImag(10,:);

array = phased.ULA(n,d,'Element',microphone,'ArrayAxis','x','Taper',conj(taper));
c = 343; %speed of sound

%idealAngle=-180:1:180;
idealAngle=-180:1:180;
f= 3150;
f1=1000;
f2=6300;
Ddata= directivity(array,f,idealAngle,'PropagationSpeed',c);
Ddata1= directivity(array,f1,idealAngle,'PropagationSpeed',c);
Ddata2= directivity(array,f2,idealAngle,'PropagationSpeed',c);
% random image for white background
a = rand(100,100);

% initial axis
h = axes('position',[0  0  1  1]);
colormap white

%plotting image
f1 = imagesc(a);
h2 = axes('position',[0  0  1  1]);

%polarpattern(h2, interAngles,interDB180Deg,idealAngle,Ddata,'NormalizeData',1,'LineWidth',4,'FontSize',24,'MagnitudeLim',[-100 0])
polarpattern(h2,idealAngle,Ddata1,idealAngle,Ddata, idealAngle,Ddata2,'NormalizeData',1,'LineWidth',2,'FontSize',22,'MagnitudeLim',[-100 0])
% removing background of polar plot - so image shows through
legend({'1.00kHz', '3.15kHz','6.30kHz'}, 'FontSize', 22)
ph=findall(h2,'type','patch');
set(ph,'FaceColor','white')

%% 0 degrees

taper = weightTableImag(1,:);

array = phased.ULA(n,d,'Element',microphone,'ArrayAxis','x','Taper',conj(taper));
c = 343; %speed of sound

%idealAngle=-180:1:180;
idealAngle=-180:1:180;
f= 3150;
f1=1000;
f2=6300;
Ddata= directivity(array,f,idealAngle,'PropagationSpeed',c);
Ddata1= directivity(array,f1,idealAngle,'PropagationSpeed',c);
Ddata2= directivity(array,f2,idealAngle,'PropagationSpeed',c);
% random image for white background
a = rand(100,100);

% initial axis
h = axes('position',[0  0  1  1]);
colormap white

%plotting image
f1 = imagesc(a);
h2 = axes('position',[0  0  1  1]);

%polarpattern(h2, interAngles,interDB180Deg,idealAngle,Ddata,'NormalizeData',1,'LineWidth',4,'FontSize',24,'MagnitudeLim',[-100 0])
polarpattern(h2,idealAngle,Ddata1,idealAngle,Ddata, idealAngle,Ddata2,'NormalizeData',1,'LineWidth',2,'FontSize',22,'MagnitudeLim',[-100 0])
% removing background of polar plot - so image shows through
legend({'1.00kHz', '3.15kHz','6.30kHz'}, 'FontSize', 22)
ph=findall(h2,'type','patch');
set(ph,'FaceColor','white')

%% 60 degrees
taper = weightTableImag(7,:);

array = phased.ULA(n,d,'Element',microphone,'ArrayAxis','x','Taper',conj(taper));
c = 343; %speed of sound

%idealAngle=-180:1:180;
idealAngle=-180:1:180;
f= 3150;
f1=1000;
f2=6300;
Ddata= directivity(array,f,idealAngle,'PropagationSpeed',c);
Ddata1= directivity(array,f1,idealAngle,'PropagationSpeed',c);
Ddata2= directivity(array,f2,idealAngle,'PropagationSpeed',c);
% random image for white background
a = rand(100,100);

% initial axis
h = axes('position',[0  0  1  1]);
colormap white

%plotting image
f1 = imagesc(a);
h2 = axes('position',[0  0  1  1]);

%polarpattern(h2, interAngles,interDB180Deg,idealAngle,Ddata,'NormalizeData',1,'LineWidth',4,'FontSize',24,'MagnitudeLim',[-100 0])
polarpattern(h2,idealAngle,Ddata1,idealAngle,Ddata, idealAngle,Ddata2,'NormalizeData',1,'LineWidth',2,'FontSize',22,'MagnitudeLim',[-100 0])
% removing background of polar plot - so image shows through
legend({'1.00kHz', '3.15kHz','6.30kHz'}, 'FontSize', 22)
ph=findall(h2,'type','patch');
set(ph,'FaceColor','white')