function MOI = getMOI(rocket, motor, cg, yi)

% Assume rocket is symmetric about Z axis and each component is symmetric
% Get ZZ component
IzzFins = rocket.fin.numFins*(1/12*rocket.structure.finMass*(rocket.fin.width^2 + rocket.fin.thickness^2) + ...
    rocket.structure.finMass*(rocket.structure.tubeDiameter/2 + rocket.fin.width/2)^2);
IzzTube = rocket.structure.tubeMass*(rocket.structure.tubeDiameter/2)^2;
IzzMotor = .5*(motor.dryMass + yi(13))*(rocket.structure.tubeDiameter/2)^2;

%%%%%%%%%%%%%%%%%% Change this to be more accurate %%%%%%%%%%%%%%%%%%%%%%
% Get YY component
IyyFins = IzzFins*5;
IyyTube = IzzTube*5;
IyyMotor = IzzMotor*5;


% Get ZZ component
IxxFins = IzzFins*5;
IxxTube = IzzTube*5;
IxxMotor = IzzMotor*5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
MOI = [IxxFins+IxxTube+IxxMotor 0 0;
       0 IyyFins+IyyTube+IyyMotor 0;
       0 0 IzzFins+IzzTube+IzzMotor];
end