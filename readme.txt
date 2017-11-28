Author: Gevashkar Rampersadh
Supervisor: Robyn Verrinder

This repository pertains to the work generated for the completion of a MSc. thesis at the University of Cape Town.

The main function file, WGSim.m, is a MATLAB function which either generates or locates the sea state input to the system and runs the Simulink simulation, WG0920_3D, for the Wave Glider model. 

The following documentation gives a breakdown of all the utilized functions.

AddedMass:
Generate the specific added mass and coriolis for the float and glider based on positions(p), velocities(v),linear body-fixed float velocity(vfb), linear body-fixed glider velocity(vgb), and boolean stating whether float is in the water(toggleFloatAddedMass). The added mass values for the float and glider are passed to generated functions from createEquations to correctly assign the Mass and Coriolis and centripetal force.
[Ma,Ca] = AddedMass(p,v,vfb,vgb,toggleFloatAddedMass)

addedMC:
Generate the generalised added mass and coriolis matrices in the body-frame based on the diagonal hydrodynamic force co-efficients (Xu,Yv,Zw,Kp,Mq,Nr) which are assumed positive and the body-fixed velocity vector (vb) where vb = [u;v;w;p;q;r].
[M,C] = addedMC(Xu,Yv,Zw,Kp,Mq,Nr,vb)

CenVol:
Generate the centroid and volume of a convex shape defined by its vertices(P), position(pos), and an intersection surface(surface = [dZdx,dZdy,dz]) where dZdx is the gradient in the x direction, dZdy is the gradient in the y direction and dz is the z offset. The function returns the centroid and volume of the shape as well as the centriod and volume of the portion of the shape below the defined surface where a positive down z axis is used.
[C,Vt,Cu,Vut,P2] = CenVol(P,pos,surface)

createEquation:
Generates the rigid body equations of motion for the 3D Wave Glider system. The system is defined by three bodies: the float(f),  the tether(t) and the glider(g). The script generates a: Mass matrix(M), Coriolis matrix(C), Restorative matrix(G),simple damping matrix(D), Added Mass matrices for the float and glider, and Coriolis and centripetal force matrices for the float and glider. The system has 12 generalised coordinates, q = [xf;yf;zf;phif;thetaf;psif;phit;thetat;phig;thetag;psig;thetaa]. The restorative matrix deals only with gravitational restorative forces for the system and the simple damping matrix is proportional to mass and for each coordinate.

DH:
Generates the DH transform(T) from a given frame(start) to a given frame(finish) based on the DH parameter matrix(A).
[T] = DH(A,start,finish)

DragFloat:
Function to determine the linear and angular drag(Dl,Da) for the float based on the system velocities(dq) and the linear body-fixed float velocities(vfb).
[Dl,Da] = DragFloat(dq,vfb)

DragGlider:
Function to determine the linear and angular drag(Dl,Da) for the glider based on the system velocities(dq) and the linear body-fixed glider velocities(vfb).
[Dl,Da] = DragGlider(dq,vgb)

FirstOrderResponseForGlider:
Simulink model for control design of first order approximation of glider yaw system.

FloatBuoy:
Function to handle generating the volume(Vol) and centroid(Cu) of the non-convex float hull section below a surface based on convex volumes. The function takes in the generalised coordinates of the system(q) and the parameters for the surface(surface = [dZdx;dZdy;dz]).
[Vol,Cu] = FloatBuoy(q,surface)

generalisedForce:
Generalised force acting at arbitrary point in system. The script generates a function get_QGen, which takes in the generalised coordinates, q, the generalised velocities, dq, the location of the force, r, and the components of the force, F.

generalisedForceFloat:
Generalised force on float. The script generates a function get_QGenFloat, which takes in the generalised coordinates, q, the generalised velocities, dq, the location of the force in the float body-frame, r, and the components of the force, F.

generalisedForceGlider:
Generalised force on glider. The script generates a function get_QGenGlider, which takes in the generalised coordinates, q, the generalised velocities, dq, the location of the force in the glider body-frame, r, and the components of the force, F.

generalisedForceTether:
Generalised force on tether. The script generates a function get_QGenTether, which takes in the generalised coordinates, q, the generalised velocities, dq, the location of the force in the tether body-frame, r, and the components of the force, F.

GenSea:
Function to generate local sea around glider based on sinusoidal sea state. The position of the float is used as inputs and the sea surrounding the float is defined by the heights of the sea.
[plane,Xsea,Ysea] = GenSea(px,py,dt,SeaParam)

get_model_param:
Function to store and transfer system parameters such that there exist a single location for all parameters pertaining to the system.
param = get_model_param

getGenForce:
Function the handle a force(F = [Fx;Fy;Fz]) acting on a point on the system given by offset. offset is defined as a 9 element vector with each set of 3 elements relating to the position of the force on the section in the order: float, tether, glider. state determines which body the force acts on with: 0 being an arbitrary force, 1 acting on the float, 2 acting on the tether, and 3 acting on the glider.
[Q] = getGenForce(q,dq,F,offset,state)

getSpringConstant:
Function to determine spring constant for hydrofoils such that the angle of the hydrofoils is limited.
[k] = getSpringConstant(sl,thetaa,thetag)

GliderResponseModel:
Simulink model to design yaw rate controller.

GliderYawResponseModel:
Simulink model that verifies the validity of the controller generated on the first order approximation model for the system utilizing yaw rate gains from simulation.

HydrofoilForce:
Function to determine the hydrofoil forces and moments in the glider body-frame. The hydrofoil is limited to the mechanical limits of between 40 degrees and -20 degrees. The equations for the forces are depend on multiple variables.
[F] = HydrofoilForce(vx,vz,fa,v)

rfMatrix:
Function to determine the DH frame rotation transform from R(i) to R(i-1) for a single set of DH parameters(A). [R] = rFMatrix(A).
[R] = rFMatrix(A)

RudderForce:
Rudder forces on glider based on speed(body-fixed glider u in m/s) and angle(rudder angle in rad).
[F] = RudderForce(vx,ra).

symMat:
Imported code to allow for referencing of symbolic matrices in MATLAB.

WG0920_3D:
Simulink model of Wave Glider system.

WGSim:
Wave Glider simulation function. This scripts runs the Wave Glider Simulink model, where the simulation parameters can be varied, and generates a video output.

WGSimHelper:
A script to allow for batch runs of WGSim.
























