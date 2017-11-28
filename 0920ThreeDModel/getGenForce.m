% Function the handle a force(F = [Fx;Fy;Fz]) acting on a point on the
% system given by offset. offset is defined as a 9 element vector with each
% set of 3 elements relating to the position of the force on the section in
% the order: float, tether, glider. state determines which body the force
% acts on with: 0 being an arbitrary force, 1 acting on the float, 2 acting
% on the tether, and 3 acting on the glider.
% [Q] = getGenForce(q,dq,F,offset,state)
function [Q] = getGenForce(q,dq,F,offset,state)

if nargin == 4
    state = 0;
end

switch state
    case 0
        Q = get_QGen(q,dq,F,offset);
    case 1 % Force on floater
        Q = get_QGenFloat(q,dq,F,offset(1:3));
    case 2 % Force on tether
        Q = get_QGenTether(q,dq,F,offset(4:6));
    case 3 % Force on glider
        Q = get_QGenGlider(q,dq,F,offset(7:9));
end

end