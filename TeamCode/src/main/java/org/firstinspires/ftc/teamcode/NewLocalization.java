package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;


/**
 * All distance sensors used for localization are in an array with these values
 * 0=left
 * 1=right
 * 2=rear
 * 3=left
 */
@TeleOp(name="NewLocal")
public class NewLocalization extends LinearOpMode {

    final static double[] distanceThresh ={7, 7, 0, 6.5};


    DistanceSensor[] sensors;
    Servo rightClaw, leftClaw;

    double X = 0, Y = 0;
    BNO055IMU imu;
    double angleError;
    MecanumDrive mecanums;
    Encoders encoders;

    enum angleQuad{
        I,II,III,IV
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap, false);
        while(opModeIsActive()) {
            updatePosition();
            telemetry.update();
        }
    }

    void updatePosition(){
        angleQuad aQuad;
        double angle = getAngle();
        double angleDif;
        if(angle >= 45 && angle < 135){
            angleDif = Math.abs(angle - 90);
            aQuad = angleQuad.II;
        }
        else if(angle >= 135 && angle < 225){
            angleDif = Math.abs(angle - 180);
            aQuad = angleQuad.III;
        }
        else if(angle >= 225 && angle < 315){
            angleDif = Math.abs(angle - 270);
            aQuad = angleQuad.IV;
        }
        else{
            aQuad = angleQuad.I;
            if(angle >= 315){
                angleDif = 360 - angle;
            }
            else{
                angleDif = Math.abs(angle);
            }
        }

        telemetry.addData("Angle Diff", angleDif);

        double[] distance = new double[4];
        for(int i =0; i < sensors.length; i++){
            distance[i] = sensors[i].getDistance(DistanceUnit.INCH);
            if (distance[i] >= 80) {
                distance[i] = 0;
            }
            telemetry.addData("Real " + i, distance[i]);
        }

        double[] formattedDis = new double[4];

        for(int i = 0; i < distance.length; i++){
                if(i != 2 && distance[i] != 0) {
                    formattedDis[i] = (distance[i] + distanceThresh[i]) * Math.cos(Math.toRadians(angleDif));
                    telemetry.addData("Sensor" + i, formattedDis[i]);
                }
                else if(distance[i] != 0){
                    double diagDis = 10.35;
                    formattedDis[i] = diagDis*Math.cos(Math.toRadians(angleDif)) + distance[i]*Math.cos(Math.toRadians(angleDif));;
                    telemetry.addData("Sensor" + i, formattedDis[i]);
                }
                else{
                    formattedDis[i] = 0;
                }
        }


         X = 0;
         Y = 0;

        switch(aQuad){
            case III:
            case I:
                Y = 72 - formattedDis[1] - formattedDis[0];
                break;
            case II:
            case IV:
                Y = 72 - formattedDis[2] - formattedDis[3];
                break;
        }

        switch(aQuad){
            case I:
                X = isEnabled(formattedDis[2])*(72-formattedDis[2]) + isEnabled(formattedDis[3])*(-72+formattedDis[3]);
                break;
            case III:
                X = isEnabled(formattedDis[2])*(-72+formattedDis[2]) + isEnabled(formattedDis[3])*(72-formattedDis[3]);
                break;
            case II:
                X =isEnabled(formattedDis[1])*(-72+formattedDis[1]) + isEnabled(formattedDis[0])*(72-formattedDis[0]);
                break;
            case IV:
                X =isEnabled(formattedDis[1])*(72-formattedDis[1]) + isEnabled(formattedDis[0])*(-72+formattedDis[0]);
                break;
        }

        telemetry.addData("X", X);
        telemetry.addData("Y", Y);
    }

    double getAngle(){
        double ang = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES).firstAngle + angleError;
        double newangle = 0;

        if(ang <= 0 && ang > -90){
            newangle = 90-Math.abs(ang);
        }
        else if(ang < -90){
            newangle = 450 - Math.abs(ang);
        }
        else if(ang > 0){
            newangle = 90 + Math.abs(ang);
        }

        return newangle;
    }

    void init(HardwareMap hwmap, boolean limitS){
        HardwareConfig robot = new HardwareConfig(hwmap);
        robot.initializeDriveTrain();
        robot.initializeDistanceSensors();
        robot.initializeIMU();
        robot.initializeServos();
        sensors = new DistanceSensor[4];
        if(limitS) {
            robot.initializeLimitSwitches();
            DigitalChannel limitSwitchh = robot.limitLeft;
        }

        DriveTrain dt = new DriveTrain(robot.frontLeft, robot.frontRight, robot.rearLeft, robot.rearRight);
        mecanums = new MecanumDrive(dt);
        encoders = new Encoders(dt);

        encoders.resetEncoders();

        System.arraycopy(robot.distanceSensors, 0, sensors, 0, robot.distanceSensors.length);

        leftClaw = robot.leftClaw;
        rightClaw = robot.rightClaw;

        robot.initializeIMU();

        imu = robot.imu;

        angleError = -imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES).firstAngle;
    }

    double isEnabled(double val){
        if(val > 0) return 1;
        else return 0;
    }

    void turnToAngle(double dest){
        double thresh = 1.5;
        double currentAng = getAngle();

        while(currentAng < dest + thresh || currentAng > dest - thresh){
            double speed = (-1.6/((Math.abs(currentAng-dest))+3))+0.45;
            if(speed < 0.05){
                break;
            }
            telemetry.addData("Angle", currentAng);
            telemetry.addData("Speed", speed);
            telemetry.update();
            if(dest - currentAng > currentAng - dest){
                mecanums.move(0, 0, speed);
            }
            else{
                mecanums.move(0, 0, -speed);
            }
            currentAng = getAngle();
        }
        mecanums.stopNow();
    }
    void turnToAngle(double dest, double power){
        double thresh = 1.5;
        double currentAng = getAngle();

        while(currentAng < dest + thresh || currentAng > dest - thresh){
            double speed = (-1.6/((Math.abs(currentAng-dest))+3))+0.45;
            if(speed < 0.05){
                break;
            }
            telemetry.addData("Angle", currentAng);
            telemetry.addData("Speed", speed);
            telemetry.update();
            if(dest - currentAng > currentAng - dest){
                mecanums.move(90, power, speed);
            }
            else{
                mecanums.move(90, power, -speed);
            }
            currentAng = getAngle();
        }
        mecanums.stopNow();
    }

    void moveWithEncoder(double Xdest, double Ydest){
        double Xdiff = Math.abs(Math.abs(Xdest + 72) - Math.abs(X + 72));
        double Ydiff = Math.abs(Ydest - Y);
        encoders.resetEncoders();
        double speed = 0.6;

        telemetry.clearAll();
        telemetry.addData("Xdiff, Ydiff", Xdiff + "  " + Ydiff);
        telemetry.update();
        sleep(2000);

        double inches = encoders.getInches();
        double Xdir = 0;
        double Ydir = 90;

        if(Xdest > X){
            Xdir = 180;
        }

        if(Ydest < Y){
            Ydir = 270;
        }
        while(opModeIsActive() && inches < Xdiff){
            inches = encoders.getInches();
            mecanums.absMove(Xdir, speed, getAngle());
        }
        encoders.resetEncoders();

        inches = encoders.getInches();
        while(opModeIsActive() && inches < Ydiff){
            inches = encoders.getInches();
            mecanums.absMove(Ydir, speed, getAngle());
        }
        mecanums.stopNow();
    }

}
