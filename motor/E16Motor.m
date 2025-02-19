function motordata = E16Motor()

motordata.thrust = [
   0 0
   0.01 49
   0.02 49
   0.05 46
   0.10 44
   0.20 43
   0.25 42
   0.30 41
   0.35 40
   0.40 39
   0.45 38
   0.50 37
   0.55 35
   0.60 33
   0.65 32
   0.70 31
   0.75 30
   0.80 27
   0.85 25
   0.90 20
   0.91 19
   0.93 12
   0.95  5
   0.97  1
   1.00  0]; % [s, N]

motordata.dryMass = 0.044; % kg
motordata.wetMass = .084; % kg
motordata.height = .114; % m
motordata.diameter = .029; % m
