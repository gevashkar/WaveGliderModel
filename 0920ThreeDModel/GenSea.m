% Function to generate local sea around glider based on sinusoidal sea
% state. The position of the float is used as inputs and the sea
% surrounding the float is defined by the heights of the sea.
function [plane,Xsea,Ysea] = GenSea(px,py,dt,SeaParam)

numOfSignals = SeaParam.signals.values(1);
amplitude = SeaParam.signals.values(2:1+numOfSignals);
freq = SeaParam.signals.values(2+numOfSignals:1+numOfSignals*2);
off = SeaParam.signals.values(2+numOfSignals*2);
ang = SeaParam.signals.values(3+numOfSignals*2)*pi/180;

px = reshape(px,1,1,length(px));
py = reshape(py,1,1,length(py));
g = 9.81;

res = 2^6;
dis = 2^4;
x=linspace(-dis,dis,res);
y=linspace(-dis,dis,res);
[X,Y] = meshgrid(x,y); % create rectangular mesh

dir = [cos(ang) -sin(ang);sin(ang) cos(ang)]*[off;0];
xoff = dir(1);
yoff = dir(2);

pxarr = repmat(px,size(X(:,:,1)));
pyarr = repmat(py,size(Y(:,:,1)));
xoffarr = repmat(xoff,size(pxarr));
yoffarr = repmat(yoff,size(pxarr));
Xarr = repmat(X,size(pxarr(1,1,:)));
Yarr = repmat(Y,size(pxarr(1,1,:)));
Xgen = Xarr+pxarr-xoffarr;
Ygen = Yarr+pyarr-yoffarr;
Xsea = Xarr+pxarr;
Ysea = Yarr+pyarr;
R = sqrt((Xgen).^2 + (Ygen).^2);

t(1,1,:) = 0:dt:dt*length(px)-dt;
plane = zeros(size(Xarr));

for i = 1:numOfSignals
    w = freq(i);
    A = amplitude(i);
    T = 1/w;
    k = w^2/g;
    phi = t/T;
    phiarr = repmat(phi,size(X));
    plane = plane + A*sin(k*R-phiarr);
end

end

