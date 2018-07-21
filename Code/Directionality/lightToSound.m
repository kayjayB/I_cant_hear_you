function soundFreq = lightToSound(lightFreq)
    lambdaLight = (3*10^8)/lightFreq;
    soundFreq = 343/lambdaLight;
end