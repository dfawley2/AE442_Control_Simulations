function ydot = eom(tCurr, yi, rocket, motor, ctrl, models)

trajCalcs = getTrajCalcs(tCurr, yi, rocket, motor, ctrl, models);

%% Translational Acceleration
aTrans = (trajCalcs.FGravI + trajCalcs.FThrustI + trajCalcs.FDragBodyI + ...
    sum(trajCalcs.FLiftFinI,2) + sum(trajCalcs.FDragFinI,2))/trajCalcs.massTot;

%% Angular Acceleration
% find moments in body frame to get Euler angle derivatives
MFinsI = trajCalcs.MFinsI;
MFinsB = quatVectorRotation(trajCalcs.qi2b, MFinsI);
MFinsBTot = sum(MFinsB, 2);
% ang acceleration as [roll, pitch, yaw] as 321 Euler Angles
alpha(1) = MFinsBTot(3)/trajCalcs.MOI(3,3);
alpha(2,1) = MFinsBTot(2)/trajCalcs.MOI(2,2);
alpha(3,1) = MFinsBTot(1)/trajCalcs.MOI(1,1);

ydot(1:3,1) = yi(4:6);
ydot(4:6,1) = aTrans;
ydot(7:9,1) = yi(10:12);
ydot(10:12,1) = alpha;
ydot(13) = trajCalcs.dmdt;


end