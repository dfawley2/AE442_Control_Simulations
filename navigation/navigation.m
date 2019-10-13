function nav = navigation(y,tCurr, trajCalcs, models)

switch models.navMode
    case 1 % perfect nav
        nav.posI = y(1:3);
        nav.velI = y(4:6);
        nav.EulerAngles = y(7:9);
        nav.omega = y(10:12);
        
end
end