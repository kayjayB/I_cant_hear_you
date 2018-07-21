Fs = 40000;
transferFuncEstimator = dsp.TransferFunctionEstimator...
                                 ('FrequencyRange','onesided',...
                                  'SpectralAverages',20);
L = 2^14;
scope = dsp.ArrayPlot( ...
    'PlotType','Line', ...
    'XOffset',0, ...
    'YLimits',[-120 5], ...
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