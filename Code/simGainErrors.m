frequencies = [240, 320, 400, 500, 640, 800, 1000, 1240, 1600, 2000, 2500, 3140, 4000, 5000, 6300, 8000];

for i=1:16
    
    result = find(freqx==frequencies(i));
    gain(i) = magnitudeFiltered(result) - magnitudeOriginal(result);
    error(i) = abs(audiogramdB(i) - gain(i))/audiogramdB(i)*100;
end
%%
plot(frequencies,error)
%%
tab1 = frequencies(1:8);
tab1 = [tab1; error(1:8)];

tab2 = frequencies(9:16);
tab2 = [tab2; error(9:16)];

tab = [tab1; tab2];

tab = transpose(tab);

input.data = tab;
latex = latexTable(input);