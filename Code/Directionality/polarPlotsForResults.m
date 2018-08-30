clc 
clear

angles = 0:30:360;

%angles=deg2rad(angles);

% raw0=[35.9,32.8,34.3,15.6,46.8,34.3,34.3];
% raw60=[42.1,43.7,29.6,34.3,62.5,31.2,37.5];
% raw90=[28.1,54.6,48.4,109,70.3,45.3,62.5];
% raw120=[17.1,35.9,67.1,84.3,42.1,45.3,40.6];
% raw180=[34.3,35.9,34.3,18.7,50,25,32.8];

raw0=[35.9,32.8,34.3,15.6,46.8,34.3,34.3,1,1,1,1,1,35.9];
raw60=[42.1,43.7,29.6,34.3,62.5,31.2,37.5,1,1,1,1,1,42.1];
raw90=[28.1,54.6,48.4,109,70.3,45.3,62.5,1,1,1,1,1,28.1];
raw120=[17.1,35.9,67.1,84.3,42.1,45.3,40.6,1,1,1,1,1,17.1];
raw180=[34.3,35.9,34.3,18.7,50,25,32.8,1,1,1,1,1,34.3];

rawOmni=[141,84.3,84.3,140,68.7,68.7,59.3,1,1,1,1,1,1];
rawMic=[199,159,131,146,100,140,156,1,1,1,1,1,1];

%normalize

% maxRaw0= max(raw0);
% norm0 = raw0./maxRaw0;
% 
% maxRaw60= max(raw60);
% norm60 = raw60./maxRaw60;
% 
% maxRaw90= max(raw90);
% norm90 = raw90./maxRaw90;
% 
% maxRaw120= max(raw120);
% norm120 = raw120./maxRaw120;
% 
% maxRaw180= max(raw180);
% norm180 = raw180./maxRaw180;
% 
% 
% gainDB0=mag2db(norm0);
% gainDB60=mag2db(norm60);
% gainDB90=mag2db(norm90);
% gainDB120=mag2db(norm120);
% gainDB180=mag2db(norm180);
% 
% figure
% polarpattern(angles,gainDB0);
% figure
% polarpattern(angles,gainDB60);
% figure
% polarpattern(angles,gainDB90);
% figure
% polarpattern(angles,gainDB120);
% figure
% polarpattern(angles,gainDB180);

f = 3340;
n = 4; %no of microphones
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
taper = weightTableImag(10,:);

array = phased.ULA(n,d,'Element',microphone,'ArrayAxis','x','Taper',conj(taper));
c = 343; %speed of sound

%idealAngle=-180:1:180;
idealAngle=-180:1:180;
f= 3340;
f1=1000;
f2=6300;
Ddata= directivity(array,f,idealAngle,'PropagationSpeed',c);
Ddata1= directivity(array,f1,idealAngle,'PropagationSpeed',c);
Ddata2= directivity(array,f2,idealAngle,'PropagationSpeed',c);

%%
%interRaw0=interp(gainDB0,2);
% maxInterRaw0= max(interRaw0);
% normInter0 = interRaw0./maxInterRaw0;
% normInter0=abs(normInter0);
% gainDBInter0=mag2db(normInter0);
interAngles = linspace(0,360,26);
% figure
% polarpattern(interAngles,interRaw0);


db0Deg=zeros(1,length(raw0));
for i=1:length(raw0)
    db0Deg(1,i)=20*log(raw0(i));
    db60Deg(1,i)=20*log(raw60(i));
    db90Deg(1,i)=20*log(raw90(i));
    db120Deg(1,i)=20*log(raw120(i));
    db180Deg(1,i)=20*log(raw180(i));
    dbOmni(1,i)=20*log(rawOmni(i));
    dbMic(1,i)=20*log(rawMic(i));
end

interDB0Deg=interp(db0Deg,2);
interDB60Deg=interp(db60Deg,2);
interDB90Deg=interp(db90Deg,2);
interDB120Deg=interp(db120Deg,2);
interDB180Deg=interp(db180Deg,2);
interDBOmni=interp(dbOmni,2);
interDBMic=interp(dbMic,2);

interDB0Deg(length(interDB0Deg))=interDB0Deg(1);
interDB60Deg(length(interDB60Deg))=interDB60Deg(1);
interDB90Deg(length(interDB90Deg))=interDB90Deg(1);
interDB120Deg(length(interDB120Deg))=interDB120Deg(1);
interDB180Deg(length(interDB180Deg))=interDB180Deg(1);
interDBOmni(length(interDBOmni))=interDBOmni(1);
interDBMic(length(interDBMic))=interDBMic(1);

% random image for white background
a = rand(100,100);

% initial axis
h = axes('position',[0  0  1  1]);
colormap white

%plotting image
f1 = imagesc(a);
h2 = axes('position',[0  0  1  1]);
polarpattern(h2, interAngles,interDB90Deg,idealAngle,Ddata,'NormalizeData',1,'LineWidth',2,'FontSize',22,'MagnitudeLim',[-100 0])
%polarpattern(h2,idealAngle,Ddata1,idealAngle,Ddata, idealAngle,Ddata2,'NormalizeData',1,'LineWidth',2,'FontSize',14,'MagnitudeLim',[-100 0])
%polarpattern(h2, interAngles,interDBOmni,interAngles,interDBMic,'NormalizeData',1,'LineWidth',2,'FontSize',22,'MagnitudeLim',[-100 0])
% removing background of polar plot - so image shows through
legend({'DAC', 'Microphone',}, 'FontSize', 22)
ph=findall(h2,'type','patch');
set(ph,'FaceColor','white')

% %%
% h2 = axes('position',[0  0  1  1]);
% figure
% polarpattern(h2, interAngles,interDB60Deg)
% h2 = axes('position',[0  0  1  1]);
% figure
% polarpattern(h2,interAngles,interDB90Deg)
% h2 = axes('position',[0  0  1  1]);
% figure
% polarpattern(h2,interAngles,interDB120Deg)
% 
% %%
% h2 = axes('position',[0  0  1  1]);
% figure
% polarpattern(h2,interAngles,interDB180Deg,idealAngle,Ddata)
% h2 = axes('position',[0  0  1  1]);
% figure
% polarpattern(h2,idealAngle,Ddata)

%%
%error stuffs

%normalize
db0DegMax=max(db0Deg);
db60DegMax=max(db60Deg);
db90DegMax=max(db90Deg);
db120DegMax=max(db120Deg);
db180DegMax=max(db180Deg);
measured0=zeros(length(db0Deg));
measured60=zeros(length(db60Deg));
measured90=zeros(length(db90Deg));
measured120=zeros(length(db120Deg));
measured180=zeros(length(db180Deg));

measured0=db0Deg-db0DegMax;
measured60=db60Deg-db60DegMax;
measured90=db90Deg-db90DegMax;
measured120=db120Deg-db120DegMax;
measured180=db180Deg-db180DegMax;

normDdata=zeros(length(db0Deg));

DdataMax=max(Ddata);

normDdata=Ddata-DdataMax;

a=0:30:180;

normDdata=db2mag(normDdata);
measured90=db2mag(measured90);

figure
plot(a,normDdata,a,measured90)
% hold on;
% plot(a,measured90)
% hold off;

avError= abs(((measured90-transpose(normDdata))./transpose(normDdata)).*100)


%%
%error calculations performed in notebook in conjunction with code

averageError = [46.55,30.675,12.725,22.6625,51.725];

dirAng=[0,30,60,90,120];
figure
bar(dirAng,averageError)
xlabel('Directionality angle');
ylabel('Error (%)')













