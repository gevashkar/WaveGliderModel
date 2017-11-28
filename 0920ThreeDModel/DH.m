% Generates the DH transform(T) from a given frame(start) to a given
% frame(finish) based on the DH parameter matrix(A).
% [T] = DH(A,start,finish)
function [T] = DH(A,start,finish)

T = eye(4);

for i = start+1:finish
    
    T = T*rFMatrix(A(i,:));
    
end

end