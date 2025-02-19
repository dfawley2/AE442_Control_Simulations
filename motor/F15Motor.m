function motordata = F15Motor()

motordata.thrust = [
0 0 
0.063 2.127
0.118 4.407
0.158 8.359
0.228 13.68
0.340 20.82
0.386 26.75
0.425 25.38
0.481 22.19
0.583 17.93
0.883 16.11
1.191 14.59
1.364 15.35
1.569 15.65
1.727 14.74
2.00 14.28
2.39 13.68
2.68 13.08
2.96 13.07
3.25 13.05
3.35 13.0
3.39 7.30
3.40 0.00]; % [s, N]

motordata.dryMass = 0.043; % kg
motordata.wetMass = .103; % kg
motordata.height = .114; % m
motordata.diameter = .029; % m
