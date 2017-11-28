% Function to determine spring constant for hydrofoils such that the angle
% of the hydrofoils is limited.
function k = getSpringConstant(sl,thetaa,thetag)

sl = sqrt(sl.'*sl);
l2 = 0.07675;
l1 = 0.1375;
k0 = 100;
s = 150000;

if((thetaa-thetag>=0)&&(sl>l1))
    k = k0+s*(exp(sl-l1)-1);
elseif((thetaa-thetag<0)&&(sl>l2))
    k = k0+s*(exp(sl-l2)-1);
else
    k = k0;
end
    
end