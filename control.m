function ctrl = control(motor,rocket,ctrl,nav,tCurr)

if ~ctrl.igniteMotor
    if tCurr >= ctrl.tIgnite
        ctrl.igniteMotor = 1;
    end
end

end