package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Utils {
    RobotClass robot;
    public Utils(RobotClass robot){
        this.robot = robot;
    }
    public double limitCurrentDraw(double maximumCurrentDraw, RobotClass.MOTORS motor){
        double correctedPower = 0;
        robot.Motors.get(motor).setCurrentAlert(1, CurrentUnit.AMPS);
        if(maximumCurrentDraw < robot.Motors.get(motor).getCurrent(CurrentUnit.MILLIAMPS)){
            correctedPower = robot.Motors.get(motor).getPower() - .1;
        }
        return correctedPower;
    }
}
