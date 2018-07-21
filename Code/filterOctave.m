function output = filterOctave(oneThirdOctaveFilterBank,buffer, audiogram)

    for i=1:16
        oneThirdOctaveFilter = oneThirdOctaveFilterBank{i};
        yw(:,i) = oneThirdOctaveFilter(buffer);
        yw(:,i) = yw(:,i) * audiogram(i);
    end
%     for i=17:19
%         oneThirdOctaveFilter = oneThirdOctaveFilterBank{i};
%         yw(:,i) = oneThirdOctaveFilter(buffer);
%     end
   output = yw(:,1)+ yw(:,2)+ yw(:,3)+ yw(:,4) +yw(:,5)+ yw(:,6)+ ...
       yw(:,7)+ yw(:,8)+ yw(:,9)+ yw(:,10)+ yw(:,11)+ yw(:,12)+ ...
       yw(:,13)+ yw(:,14)+ yw(:,15)+ yw(:,16);
   %+ yw(:,17) + yw(:,18)+ ... 
   %    yw(:,19);

end

