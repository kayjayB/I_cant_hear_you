function [s, flag ] = SerialComs( comPort )
% adapted from:
% http://www.matlabarduino.org/accelerometers.html

flag=1 ;

% Set up serial communication
s= serial(comPort) ;
set(s, 'DataBits', 8) ;
set(s, 'StopBits', 1) ;
set(s, 'BaudRate', 230400) ;
set(s, 'Parity', 'None') ;
fopen(s);

% % Check that data is transmitting
% a='b';
% while (a ~='a')
%     a=fread(s,1,'uchar');
% end
% if (a=='a')
%     disp('Serial has been read');
% end
% fprintf(s, '%c', 'a' );
mbox= msgbox('Serial Communication Setup');
uiwait(mbox); 
fscanf(s, '%f') ;
end
