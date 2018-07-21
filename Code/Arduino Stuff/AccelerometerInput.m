function x =AccelerometerInput(acc)
% adapted from:
% http://www.matlabarduino.org/accelerometers.html

    fprintf(acc.s, 'R');
    
    % read in from arduino in unsigned integer format
    data(1)=fscanf(acc.s,'%f');
    
    x=data(1);

end 