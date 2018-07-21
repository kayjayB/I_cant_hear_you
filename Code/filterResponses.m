%% All the filters with their magnitude response
filter24 = filterBand24;
filter25 = filterBand25;
filter26 = filterBand26;
filter27 = filterBand27;
filter28 = filterBand28;
filter29 = filterBand29;
filter30 = filterBand30;
filter31 = filterBand31;
filter32 = filterBand32;
filter33 = filterBand33;
filter34 = filterBand34;
filter35 = filterBand35;
filter36 = filterBand36;
filter37 = filterBand37;
filter38 = filterBand38;
filter39 = filterBand39;

fvtool(filter24, 1, filter25, 1, filter26, 1, filter27, 1, filter28, ...
 filter29, 1, filter30, 1, filter31, 1, filter32, 1, filter33, ...
 filter34, 1, filter35, 1, filter36, 1, filter37, 1, filter38, 1, filter39);

%%
fvtool( filter34, 1, filter35, 1, filter36, 1);