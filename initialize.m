function result = initialize(rocket, ctrl, nav, trajCalcs)

% Trajectory
result.traj.tCurr = 0;
result.traj.posI = rocket.y0(1:3);
result.traj.velI = rocket.y0(4:6);
result.traj.EulerAngles = rocket.y0(7:9);
result.traj.omega = rocket.y0(10:12);
result.traj.propMass = rocket.y0(13);
result.traj.thrustI = trajCalcs.FThrustI;
result.traj.gravityI = trajCalcs.FGravI;
result.traj.dragBodyI = trajCalcs.FDragBodyI;
result.traj.dragFin1I = trajCalcs.FDragFinI(:,1);
result.traj.dragFin2I = trajCalcs.FDragFinI(:,2);
result.traj.dragFin3I = trajCalcs.FDragFinI(:,3);
result.traj.liftFin1I = trajCalcs.FLiftFinI(:,1);
result.traj.liftFin2I = trajCalcs.FLiftFinI(:,2);
result.traj.liftFin3I = trajCalcs.FLiftFinI(:,3);
result.traj.momentFin1I = trajCalcs.MFinsI(:,1);
result.traj.momentFin2I = trajCalcs.MFinsI(:,2);
result.traj.momentFin3I = trajCalcs.MFinsI(:,3);
result.traj.rho = trajCalcs.rho;
result.traj.qi2b = trajCalcs.qi2b;
result.traj.clFin = trajCalcs.clFin';
result.traj.cdFin = trajCalcs.cdFin';
result.traj.aoa = trajCalcs.alpha';

if rocket.fin.numFins > 4
    result.traj.dragFin4I = trajCalcs.FDragFinI(:,4);
    result.traj.liftFin4I = trajCalcs.FLiftFinI(:,4);
    result.traj.momentFin4I = trajCalcs.MFinsI(:,4);
end

% Navigation
result.nav.posI = nav.posI;
result.nav.velI = nav.velI;
result.nav.EularAngles = nav.EulerAngles;
result.nav.omega = nav.omega;

% Control
result.ctrl.igniteCotor = ctrl.igniteMotor;
result.ctrl.tIgnite = ctrl.tIgnite;

result.term.endCondition = 0;




end