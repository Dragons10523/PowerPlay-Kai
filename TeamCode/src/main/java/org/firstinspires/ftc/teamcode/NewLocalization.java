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

    final static double RobotWidth = 18;
    final static double RobotHeight = 17;
    final static double[] distanceThresh ={7, 7, 0, 6.5};


    DistanceSensor[] sensors;

    double X = 0, Y = 0;
    BNO055IMU imu;
    double angleError;

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

        Distance[] rawDistance = {new Distance(0), new Distance(0), new Distance(0), new Distance(0)};
        for(int i =0; i < sensors.length; i++){
                rawDistance[i].setDis(sensors[i].getDistance(DistanceUnit.INCH));
                telemetry.addData("RawDistance", rawDistance[i].getDis());
                if (rawDistance[i].getDis() >= 80) {
                    rawDistance[i].setDis(0);
            }
        }
        telemetry.addData("Rear", rawDistance[2].getDis());

        double[] distance = {rawDistance[0].getDis(), rawDistance[1].getDis(), rawDistance[2].getDis(), rawDistance[3].getDis()};
        double[] formattedDis = new double[4];

        for(int i = 0; i < distance.length; i++){
                if(i != 2) {
                    formattedDis[i] = (distance[i] + distanceThresh[i]) * Math.cos(Math.toRadians(angleDif));
                    telemetry.addData("Sensor" + i, formattedDis[i]);
                }
                else{
                    double diagDis = 10.35;
                    formattedDis[i] = diagDis*Math.cos(Math.toRadians(angleDif)) + distance[i]*Math.cos(Math.toRadians(angleDif));;
                    telemetry.addData("Sensor" + i, formattedDis[i]);
                }
        }

        final double lastX;
        final double lastY;


        double X;
        double Y;

        switch(aQuad){
            case III:
            case I:
                Y = 72 - formattedDis[2] - formattedDis[3];
                break;
            case II:
            case IV:
                Y = 72 - formattedDis[1] - formattedDis[4];
                break;
        }

        switch(aQuad){
            case I:
            case III:

        }



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
        sensors = new DistanceSensor[4];
        if(limitS) {
            robot.initializeLimitSwitches();
            DigitalChannel limitSwitchh = robot.limitLeft;
        }

        DriveTrain dt = new DriveTrain(robot.frontLeft, robot.frontRight, robot.rearRight, robot.rearLeft);
        MecanumDrive mecanums = new MecanumDrive(dt);
        Encoders encoders = new Encoders(dt);

        encoders.resetEncoders();

        System.arraycopy(robot.distanceSensors, 0, sensors, 0, robot.distanceSensors.length);

        Servo leftClaw = robot.leftClaw;
        Servo rightClaw = robot.rightClaw;

        robot.initializeIMU();

        imu = robot.imu;

        angleError = -imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES).firstAngle;
    }


}
