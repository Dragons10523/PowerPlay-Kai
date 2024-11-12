package org.firstinspires.ftc.teamcode.Drivetrain;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotClass;

import java.util.Map;

public abstract class AbstractOmniDrivetrain {

    double impulseRotation;
    RobotClass robotClass;
    Map<RobotClass.MOTORS, DcMotorEx> Motors;

    public AbstractOmniDrivetrain(Map<RobotClass.MOTORS, DcMotorEx> Motors, double impulseRotation, RobotClass robotClass) {

        this.robotClass = robotClass;
        this.impulseRotation = impulseRotation;
        this.Motors = Motors;

    }


    @SuppressLint("DefaultLocale")
    public void mecanumDriveGlobal(double leftY, double leftX, double turn, double heading_RADIANS) {
        // drive == y strafe == x

        //starting value off by 90 degrees; 270 == -90
//        heading_RADIANS += Math.toRadians(90);
        double rotX = leftX * Math.cos(-heading_RADIANS) - leftY * Math.sin(-heading_RADIANS);
        double rotY = leftX * Math.sin(-heading_RADIANS) + leftY * Math.cos(-heading_RADIANS);
        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(turn), 1);
        // normalizes ranges from 0 to 1
        Motors.get(RobotClass.MOTORS.FRONT_LEFT).setPower((rotY + rotX - turn) / denominator);
        Motors.get(RobotClass.MOTORS.BACK_LEFT).setPower((rotY - rotX - turn) / denominator);
        Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower((rotY - rotX + turn) / denominator);
        Motors.get(RobotClass.MOTORS.BACK_RIGHT).setPower((rotY + rotX + turn) / denominator);

    }
    public void mecanumDriveLocal(double leftY, double leftX, double turn){

        double denominator = Math.max(Math.abs(leftX) + Math.abs(leftY) + Math.abs(turn), 1);

        Motors.get(RobotClass.MOTORS.FRONT_LEFT).setPower((leftY + leftX - turn) / denominator);
        Motors.get(RobotClass.MOTORS.BACK_LEFT).setPower((leftY - leftX - turn) / denominator);
        Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower((leftY - leftX + turn) / denominator);
        Motors.get(RobotClass.MOTORS.BACK_RIGHT).setPower((leftY + leftX + turn) / denominator);
    }

}