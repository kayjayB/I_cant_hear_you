function [output] = filterFrames(buffer, audiogram)

     a = filterBand24Function(buffer);
     a = a * audiogram(1);
     b = filterBand25Function(buffer);
     b = b * audiogram(2);
     c = filterBand26Function(buffer);
     c = c * audiogram(3);
     d = filterBand27Function(buffer);
     d = d * audiogram(4);
     e = filterBand28Function(buffer);
     e = e * audiogram(5);
     f = filterBand29Function(buffer);
     f = f * audiogram(6);
     g = filterBand30Function(buffer);
     g = g * audiogram(7);
     h = filterBand31Function(buffer);
     h = h * audiogram(8);
     i = filterBand32Function(buffer);
     i = i * audiogram(9);
     j = filterBand33Function(buffer);
     j = j * audiogram(10);
     k = filterBand34Function(buffer);
     k = k * audiogram(11);
     l = filterBand35Function(buffer);
     l = l * audiogram(12);
     m = filterBand36Function(buffer);
     m = m * audiogram(13);
     n = filterBand37Function(buffer);
     n = n * audiogram(14);
     o = filterBand38Function(buffer);
     o = o * audiogram(15);
     p = filterBand39Function(buffer);
     p = p * audiogram(16);
    

     output = a+ b+ c+ d +e+ f+ g+ h+ i+ j+ k+ l+ m+ n+ o+ p;
end

