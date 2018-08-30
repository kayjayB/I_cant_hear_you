% y = yw(:,1)+ yw(:,2)+ yw(:,3)+ yw(:,4) +yw(:,5)+ yw(:,6)+ yw(:,7)+ yw(:,8)+ yw(:,9)+ yw(:,10)+ yw(:,11)+ yw(:,12)+ yw(:,13)+ yw(:,14)+ yw(:,15)+ yw(:,16);
% z  = transferFuncEstimator(repmat(buffer2,1,17),[ yw(:,1), yw(:,2), yw(:,3), yw(:,4) ,yw(:,5), yw(:,6), yw(:,7), yw(:,8), yw(:,9), yw(:,10), yw(:,11), yw(:,12), yw(:,13), yw(:,14), yw(:,15), yw(:,16), y]);
% 
% freqNew = transpose(freqx2);
% z = 20*log10(abs(z));
% plot(freqNew, z)
Fs = 40000;
L = 2^14;
transferFuncEstimator = dsp.TransferFunctionEstimator...
                                 ('FrequencyRange','onesided',...
                                  'SpectralAverages',20);
oneThirdOctaveFilterBank = createOneThirdOctaveFilters(14, Fs);

for counter = 1:10
buffer = randn(L,1);
freqx2 = linspace(0,Fs/2,length(buffer)/2+1);
for i=1:16
    oneThirdOctaveFilter = oneThirdOctaveFilterBank{i};
    yw(:,i) = oneThirdOctaveFilter(buffer);
end
y = yw(:,1)+ yw(:,2)+ yw(:,3)+ yw(:,4) +yw(:,5)+ yw(:,6)+ yw(:,7)+ yw(:,8)+ yw(:,9)+ yw(:,10)+ yw(:,11)+ yw(:,12)+ yw(:,13)+ yw(:,14)+ yw(:,15)+ yw(:,16);
z  = transferFuncEstimator(repmat(buffer,1,17),[ yw(:,1), yw(:,2), yw(:,3), yw(:,4) ,yw(:,5), yw(:,6), yw(:,7), yw(:,8), yw(:,9), yw(:,10), yw(:,11), yw(:,12), yw(:,13), yw(:,14), yw(:,15), yw(:,16), y]);
z = 20*log10(abs(z));
plot(freqx2, z, 'LineWidth',1.5)
ylim([-150 5])
xlabel('Frequency (Hz)','FontSize',22)
ylabel('Magnitude (dB)','FontSize',22)
set(gca,'fontsize',22)
drawnow
end

