function bandOutput = timeDelay(bandInput,weightTableTimeDelay, Fs)
bandOutput = delayseq(bandInput,weightTableTimeDelay,Fs);
end

