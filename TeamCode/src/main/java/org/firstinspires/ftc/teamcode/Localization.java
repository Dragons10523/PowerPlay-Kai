package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public abstract class Localization extends Control {

    public ElapsedTime time;
    protected double theta;
    private boolean turningFlag = false;
    private double targetAngle;
    private int turnDirection;

    public void startLocalization() {
        Thread mouse = new Thread(thalatte.mouse,"mouseTrap");
        mouse.start();
        time = new ElapsedTime();
    }

    public void updateLocalization() {
        theta = thalatte.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle;
        theta %= Math.PI * 2;
        thalatte.lightsaber.setTheta(theta);
        if(turningFlag) updateTurnTo();
    }

    public void startTurnTo(double theta){
        turnDirection = 1;
        theta %= Math.PI * 2;

        if(theta - this.theta > 0) turnDirection =  1;
        if(theta - this.theta < 0) turnDirection = -1;
        if(Math.abs(theta - this.theta) > Math.PI) turnDirection = -turnDirection;

        targetAngle = theta;
        turningFlag = true;
    }

    public void updateTurnTo(){
        if((targetAngle >= theta - 0.07)&&(targetAngle <= theta + 0.07)) stopTurnTo();

        double power = 1 * turnDirection;

        if(Math.abs(targetAngle - theta) > Math.PI / 8) power =   1;
        else                                            power = 0.1;

        drive(-power,power);
    }

    public void stopTurnTo(){
        turningFlag = false;
        drive(0,0);
    }

    public void moveTo(double x, double y){
        //TODO: implement moveTo (requires localization)
    }

}