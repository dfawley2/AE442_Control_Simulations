function motordata = E31Motor()

motordata.thrust = [
    0 0
    0.02 43.824
    0.027 39.964
    0.049 26.781
    0.113 32.601
    0.193 34.739
    0.282 35.808
    0.5 34.442
    0.727 29.276
    0.771 22.743
    0.807 9.561
    0.84 3.563
    0.87 0]; % [s, N]

motordata.dryMass = 0.0409; % kg
motordata.wetMass = .052; % kg
motordata.height = .069; % m
motordata.diameter = .024; % m
