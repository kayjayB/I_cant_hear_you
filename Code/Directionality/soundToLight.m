function lightFreq = soundToLight(soundFreq)
    lambdaSound = 343/soundFreq;
    lightFreq = (3*10^8)/lambdaSound;
end


