function h = halfPolar(phi,gain,linestyle,xtickval)
% HALFPOLAR function performs the polar plot in radian angle range [0 pi] 
% using half polar coordinates
%
% HALFPOLAR(phi,gain) makes a plot with phi in radians angle range [0 pi] 
% and gain in half polar coordinates. Phi and gain could be vector or matrix
% with the same dimensions. When Phi is a vector(1XN) and vector(1XN) and
% matrix(MXN)gain values are allowable. When Phi is a matrix(MXN) and then 
% the gain should only be matrix(MXN) with the same row numbers.
%
% HALFPOLAR(phi,gain, linestyle) uses specified linestyle defined in cell 
% variable linestyle to plot phi and gain, like: linestyle = {'ko-','b--'}
%
% HALFPOLAR(phi,gain, linestyle, xtickval) plots phi and gain with the
% prescribed tick value xtickval. like: xtickval = [15 25 45 75 105]
%
% IMPORTANT NOTE:
% HALFPOLAR is designed especially for the radiated sould pressure level
% demonstration. Accordingly the gain value is designed to be dB value.
% the default xtickvalue is also designed to match the dB value range.
% So you are permitted to modify the function in order to correctly 
% demonstrate data for your specified applications.
% 
% HALFPOLAR is coded straightforward without variables checking and fault
% testing.The author don't take the responsibility for the program bug. 
%
% Example:
% phi = linspace(0,pi,20);
% gain = 80 * rand(3,length(phi)) + 40;
% h = halfPolar(phi,gain,{'k-','g-.','r--','k-'},[30 40 65 80 95 120])
% 
% See also polar;
%
% Don't heisitate to let me know the problems about the function:
% lhd06@mails.thu.edu.cn
% Departmen of Electrical Engineering, 
% Tsinghua University, Beijing, China.
%
% Reference function: polar.m provided by the matlab graphics toolbox
%
% If the program is modified and redistributed, the description above
% should be included.

min_gain = min(min(gain));
max_gain = max(max(gain));
space_val = 4;
spoke_val = 6;
spoketickval = linspace(0,pi,spoke_val+1);
if nargin < 2
    error('Not enough input arguments.');
end
  
% if nargin < 3, rangedb = [min_gain max_gain]; end
if nargin < 3  linestyle = 'auto';xtickval = linspace(round(min_gain) - 10,round(max_gain) + 10,space_val);  end
if nargin < 4, xtickval = linspace(round(min_gain) - 10,round(max_gain) + 10,space_val); end

% normalization the input value
gain_range = xtickval(end) - xtickval(1);   %obtain the gain range
gain_scale = abs((gain - xtickval(1)))/gain_range; %[0,1]

cax = newplot;
tc = get(cax,'xcolor');
ls = get(cax,'gridlinestyle');
    
th = linspace(0,pi,200);
xunit = cos(th);
yunit = sin(th);
% plot background if necessary
 if ~isstr(get(cax,'color'))
       patch('xdata',xunit,'ydata',yunit, ...
             'edgecolor',tc,'facecolor',get(gca,'color'));
end

hold on;
[m1,n1] =size(phi);
[m2,n2] =size(gain);
if(m1==1)
    for i=1:m2
        x = gain_scale(i,:) .* cos(phi);
        y = gain_scale(i,:) .* sin(phi); 
        if(strcmp(linestyle,'auto'))
            h(i) = plot(x,y,'linewidth',2); 
        else
            h(i) = plot(x,y,linestyle{i},'linewidth',2);
        end
 
    end
else
    if(m1 ~= m2)
        error('Matrix should be have the same dimension!');
    else       
        for i=1:m2
         x = gain_scale(i,:) .* cos(phi(i,:));
         y = gain_scale(i,:) .* sin(phi(i,:)); 
            if(strcmp(linestyle,'auto'))
                h(i) = plot(x,y,'linewidth',2);  %,'LineWidth', 2
            else
                h(i) = plot(x,y,linestyle{i},'linewidth',2);
            end
        end
    end
end

set(gca,'dataaspectratio',[1 1 1]);axis off;
% legend_val = legend(h(1:end),'Controlled','Uncontrolled');
% set(legend_val,'location','SouthOutside');

hold on;
%define the circle
contour_val = zeros(1,length(xtickval));
contour_val = abs((xtickval - xtickval(1))/gain_range);
for k=2:length(contour_val) %gain circles
    plot(xunit*contour_val(k), yunit*contour_val(k),ls,'color','black');
    text(contour_val(k),-0.05, sprintf('%.3g',xtickval(k)),'horiz', 'center', 'vert', 'middle'); 
    text(- contour_val(k),-0.05, sprintf('%.3g',xtickval(k)),'horiz', 'center', 'vert', 'middle'); 
end
text(contour_val(1),-0.05,sprintf('%.3g',xtickval(1)),'horiz', 'center', 'vert', 'middle');
text(-0.45,-0.15,'Sound Pressure Level(SPL) /dB');

%plot the spokes and  % annotate spokes in degrees
cst = cos(spoketickval); 
snt = sin(spoketickval);
for k = 1:length(spoketickval) - 1
    plot(cst(k) * contour_val,snt(k) * contour_val,ls,'color',tc,'linewidth',1,...
         'handlevisibility','off');
    text(1.07*cst(k),1.07*snt(k),...
        sprintf('%.3g^{o}',spoketickval(k)/pi*180),...
        'horiz', 'center', 'vert', 'middle');
end
text(1.08*cst(end),1.07*snt(end),...
        sprintf('%.3g^{o}',spoketickval(end)/pi*180),...
        'horiz', 'center', 'vert', 'middle');
hold off;