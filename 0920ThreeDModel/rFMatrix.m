% Function to determine the DH frame rotation transform from R(i) to R(i-1)
% for a single set of DH parameters(A).
% [R] = rFMatrix(A)
function [R] = rFMatrix(A)
a = A(1);
alpha = A(2);
d = A(3);
theta = A(4);

syms sA sAlpha sD sTheta
R = [cos(sTheta)             -sin(sTheta)             0           sA;
     sin(sTheta)*cos(sAlpha)  cos(sTheta)*cos(sAlpha)   -sin(sAlpha) -sin(sAlpha)*sD;
     sin(sTheta)*sin(sAlpha)  cos(sTheta)*sin(sAlpha)   cos(sAlpha)  cos(sAlpha)*sD;
     0                      0                       0           1];

if ~isempty(a) 
     R = subs(R,sA,a);
end
if ~isempty(alpha) 
     R = subs(R,sAlpha,alpha);
end
if ~isempty(d) 
     R = subs(R,sD,d);
end
if ~isempty(theta) 
     R = subs(R,sTheta,theta);
end
 
end