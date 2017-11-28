% Generate the centroid and volume of a convex shape defined by its
% vertices(P), position(pos), and an intersection surface(surface = [dZdx,dZdy,dz])
% where dZdx is the gradient in the x direction, dZdy is the gradient in
% the y direction and dz is the z offset. The function returns the centroid
% and volume of the shape as well as the centriod and volume of the portion
% of the shape below the defined surface where a positive down z axis is
% used.
% [C,Vt,Cu,Vut,P2] = CenVol(P,pos,surface)
function [C,Vt,Cu,Vut,P2] = CenVol(P,pos,surface)

zf = pos(1);
zf = [0,0,zf];
phi = pos(2);
theta = pos(3);
psi = pos(4);

dx = surface(1);
dy = surface(2);
dz = surface(3);

Rphi = [1,0,0;
        0,cos(phi),-sin(phi);
        0,sin(phi),cos(phi)];
Rtheta = [cos(theta),0,sin(theta);
          0,1,0;
          -sin(theta),0,cos(theta)];
Rpsi = [cos(psi),-sin(psi),0;
        sin(psi),cos(psi),0;
        0,0,1];
R = Rpsi*Rtheta*Rphi;
for i = 1:size(P,1)
    P(i,:) = (R*(P(i,:).')).'+zf;
end
x = P(:,1);
y = P(:,2);
z = P(:,3);
k=convhull(P);
if length(unique(k(:)))<size(P,1)
    display('Part is not convex, modifying P.');
    P = P(unique(k),:);
end
x = [x;(x(1)+x(2))/2];
y = [y;(y(1)+y(2))/2];
z = [z;(z(1)+z(2))/2];
P2 = [x(:) y(:) z(:)];
T = delaunay(P2);
n = size(T,1);
V = zeros(n,1);
C=0;
for m = 1:n
    sp = P2(T(m,:),:);
    [~,V(m)]=convhull(sp);
    C = C + V(m) * mean(sp);
end
Vt = sum(V);
C=(C./Vt);
C = C-zf;
C = R.'*C.';

np = cross([1,0,dx],[0,1,dy]);

N = zeros(size(P,1),size(P,2)+1);
count = 0;
for i = 1:size(P,1)
    if P(i,3)>(dx*P(i,1) + dy*P(i,2) + dz)
        count = count + 1;
        N(count,:) = [P(i,:),i];
    end
end
N = N(1:count,:);

if(size(N,1)>0)
    columns = (1:3);                % Adding points for plane intersection
    N2 = zeros(size(P));            % Generate empty matrix to house intersection points
    count = 0;
    for i = 1:size(N,1)
        for j = find(k==N(i,4)).'
            column = floor(j/size(k,1))+1;
            row = mod(j,size(k,1));
            if row == 0
                row = size(k,1);
            end

            check = columns(columns~=column);
            if not(sum(N(:,4)==k(row,check(1))))     % Checking if connected points are below surface
                count = count + 1;
                u = P(k(row,check(1)),:) - P(N(i,4),:);
                s1 = (np*([0;0;dz]-P(N(i,4),:).'))/(dot(np,u));
                N2(count,:) = P(N(i,4),:)+s1*u;
            end
            if not(sum(N(:,4)==k(row,check(2))))
                count = count + 1;
                u = P(k(row,check(2)),:) - P(N(i,4),:);
                s1 = (np*([0;0;dz]-P(N(i,4),:).'))/(dot(np,u));
                N2(count,:) = P(N(i,4),:)+s1*u;
            end
        end
    end
    N2 = N2(1:count,:);
    
    UN = [N(:,1:3);N2];
    UN = unique(UN,'rows');
    k2 = convhull(UN);
    UN = UN(unique(k2(:)),:);
    
    x = UN(:,1);
    y = UN(:,2);
    z = UN(:,3);
    P2 = [x(:) y(:) z(:)];
    T = delaunay(P2);
    n = size(T,1);
    V = zeros(n,1);
    Cu=0;
    for m = 1:n
        sp = P2(T(m,:),:);
        try
            [~,V(m)]=convhull(sp);
        catch ME
            [~,V(m)]=convhulln(sp,{'QJ','Pp'});
        end
        Cu = Cu + V(m) * mean(sp);
    end
    Vut = sum(V);
    Cu=Cu./Vut;
    Cu = Cu-zf;
    Cu = R.'*Cu.';

else
    P2 = [];
    Vut = 0;
    Cu = C;
end

end