%% Original Filter Bank
Fs = 40000;
transferFuncEstimator = dsp.TransferFunctionEstimator...
                                 ('FrequencyRange','onesided',...
                                  'SpectralAverages',20);
L = 2^14;
scope = dsp.ArrayPlot( ...
    'PlotType','Line', ...
    'XOffset',0, ...
    'YLimits',[-120 10], ...
    'XScale','log', ...
    'SampleIncrement', .5 * Fs/(L/2 + 1 ), ...
    'YLabel','Frequency Response (dB)', ...
    'XLabel','Frequency (Hz)', ...
    'Title','Four-Band Crossover Filter', ...
    'ShowLegend',false);
for counter = 1:10
   buffer = randn(L,1);
   % Split the signal into four bands
   
     a = filterBand24Function(buffer);

     b = filterBand25Function(buffer);

     c = filterBand26Function(buffer);

     d = filterBand27Function(buffer);

     e = filterBand28Function(buffer);

     f = filterBand29Function(buffer);

     g = filterBand30Function(buffer);

     h = filterBand31Function(buffer);

     i = filterBand32Function(buffer);

     j = filterBand33Function(buffer);

     k = filterBand34Function(buffer);

     l = filterBand35Function(buffer);

     m = filterBand36Function(buffer);
     n = filterBand37Function(buffer);

     o = filterBand38Function(buffer);

     p = filterBand39Function(buffer);

     y = a+ b+ c+ d +e+ f+ g+ h+ i+ j+ k+ l+ m+ n+ o+ p;

   z  = transferFuncEstimator(repmat(buffer,1,17),[ a, b, c, d ,e, f, g, h, i, j, k, l, m, n, o, p,y]);
   scope(20*log10(abs(z)))
end

%% Octave Filter Bank

transferFuncEstimator = dsp.TransferFunctionEstimator...
                                 ('FrequencyRange','onesided',...
                                  'SpectralAverages',20);
L = 2^14;
scope = dsp.ArrayPlot( ...
    'PlotType','Line', ...
    'XOffset',0, ...
    'YLimits',[-120 10], ...
    'XScale','log', ...
    'SampleIncrement', .5 * Fs/(L/2 + 1 ), ...
    'YLabel','Frequency Response (dB)', ...
    'XLabel','Frequency (Hz)', ...
    'Title','Four-Band Crossover Filter', ...
    'ShowLegend',false);
L = 2^14;
yw = zeros(L,Nfc);
tic
while toc < 15
    % Run for 15 seconds
    buffer = randn(L,1);
    for i=1:Nfc
        oneThirdOctaveFilter = oneThirdOctaveFilterBank{i};
        yw(:,i) = oneThirdOctaveFilter(buffer);
    end
   y = yw(:,1)+ yw(:,2)+ yw(:,3)+ yw(:,4) +yw(:,5)+ yw(:,6)+ yw(:,7)+ yw(:,8)+ yw(:,9)+ yw(:,10)+ yw(:,11)+ yw(:,12)+ yw(:,13)+ yw(:,14)+ yw(:,15)+ yw(:,16);
   z  = transferFuncEstimator(repmat(buffer,1,17),[ yw(:,1), yw(:,2), yw(:,3), yw(:,4) ,yw(:,5), yw(:,6), yw(:,7), yw(:,8), yw(:,9), yw(:,10), yw(:,11), yw(:,12), yw(:,13), yw(:,14), yw(:,15), yw(:,16), y]);
   scope(20*log10(abs(z)))

end