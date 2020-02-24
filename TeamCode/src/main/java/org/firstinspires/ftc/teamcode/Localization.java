package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


/**
 * All distance sensors used for localization are in an array with these values
 * 0=left
 * 1=right
 * 2=rear
 * 3=front
 */
@TeleOp(name="Local Flag")

public class Localization extends LinearOpMode {

    final static double[] distanceThresh = {7.5, 7.5, 7.5, 7.5};
    VuforiaLocalizer vuforia;

    public static final double COUNTS_ROTATION = 560;
    public static final double WHEEL_DIAMETER = 3.0;
    VuforiaTrackables targetsSkyStone;
    public Side side;
    HardwareConfig robot;
    DistanceSensor[] sensors;
    DistanceSensor cameraDis;
    Servo rightClaw, leftClaw;
    double X = 0, Y = 0;
    public VuforiaTrackable stoneTarget;
    boolean Xknown = true, Yknown = true;
    BNO055IMU imu;
    double angleError;
    MecanumDrive mecanums;
    Encoders encoders;

    final String VUFORIA_KEY = "AQkCE3T/////AAABmacf2GUqlUiet5GB5KP6epRTyl96EqEA9gcG1VI99J81/l4NkkwX6Nx/L7BTIL+1Z3R7yorhZ4YW1N6InBS7l7o8rKNgpbwWzBkfh3Unneq6h5xeyhbILzENlxNOVSibrronjr5199YlL3+PbMazXySVa5mnY2hXXO9CXcuv/pfEyCblbkFchA3D+Ngpkpg8CSbpkXeM6aKgGEXsnBZO7xUtE8p71aFIew1Coez3KBM5n12hoov/SdKC3O6GAcbMTX3A9wVZgACfXmw4F4Skgm/QjcfG9dOH0w7Wj3Ne6haXCVS13A2uYecamReSZZyT+BatU5nfh9t4KjRtgKGf/SMAJLIoBcbdYxBqiTmyG3WN";


    enum angleQuad {
        I, II, III, IV
    }

    enum Side {
        RED, BLUE
    }

    public void initVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);


        VuforiaTrackables trackables = this.vuforia.loadTrackablesFromAsset("Skystone");

        stoneTarget = trackables.get(0);
        stoneTarget.setName("Stoner");

        trackables.activate();
    }

    public void initVuforiaWebcam() {
        robot.initializeCameraDistance();
        cameraDis = robot.cameraDis;

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        targetsSkyStone.activate();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init(hardwareMap, false, Side.BLUE);
        while (opModeIsActive()) {
            updatePosition();
            telemetry.addData("X", X);
            telemetry.addData("Y", Y);
            telemetry.update();
        }
    }

    void updatePosition() {
        sleep(250);
        angleQuad aQuad;
        double angle = getAngle();
        double angleDif;
        if (angle >= 45 && angle < 135) {
            angleDif = Math.abs(angle - 90);
            aQuad = angleQuad.II;
        } else if (angle >= 135 && angle < 225) {
            angleDif = Math.abs(angle - 180);
            aQuad = angleQuad.III;
        } else if (angle >= 225 && angle < 315) {
            angleDif = Math.abs(angle - 270);
            aQuad = angleQuad.IV;
        } else {
            aQuad = angleQuad.I;
            if (angle >= 315) {
                angleDif = 360 - angle;
            } else {
                angleDif = Math.abs(angle);
            }
        }

        double[] distance = new double[4];
        for (int i = 0; i < sensors.length; i++) {
            distance[i] = getAvgDis(i);
            if (distance[i] >= 80) {
                distance[i] = 0;
            }
        }

        double[] formattedDis = new double[4];

        for (int i = 0; i < distance.length; i++) {
            if (/*i != 2 &&*/ distance[i] != 0) {
                formattedDis[i] = (distance[i] + distanceThresh[i]) * Math.cos(Math.toRadians(angleDif));
            } else if (distance[i] != 0) {
                double diagDis = 10.35;
                formattedDis[i] = diagDis * Math.cos(Math.toRadians(angleDif)) + distance[i] * Math.cos(Math.toRadians(angleDif));
                ;
            } else {
                formattedDis[i] = 0;
            }
        }

        switch (aQuad) {
            case III:
            case I:
                Y = 72 - formattedDis[1] - formattedDis[0];
                break;
            case II:
            case IV:
                Y = 72 - formattedDis[2] - formattedDis[3];
                break;
        }

        switch (aQuad) {
            case I:
                X = isEnabled(formattedDis[2]) * (72 - formattedDis[2]) + isEnabled(formattedDis[3]) * (-72 + formattedDis[3]);
                break;
            case III:
                X = isEnabled(formattedDis[2]) * (-72 + formattedDis[2]) + isEnabled(formattedDis[3]) * (72 - formattedDis[3]);
                break;
            case II:
                X = isEnabled(formattedDis[1]) * (-72 + formattedDis[1]) + isEnabled(formattedDis[0]) * (72 - formattedDis[0]);
                break;
            case IV:
                X = isEnabled(formattedDis[1]) * (72 - formattedDis[1]) + isEnabled(formattedDis[0]) * (-72 + formattedDis[0]);
                break;
        }
        if(side ==  Side.RED){
            X = -X;
        }
        if (Math.abs(X) == 72) {
            Xknown = false;
        }
        if (Math.abs(Y) == 72) {
            Yknown = false;
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

    void init(HardwareMap hwmap, boolean limitS, Side s) {
        robot = new HardwareConfig(hwmap);
        robot.initializeDriveTrain();
        robot.initializeDistanceSensors();
        robot.initializeIMU();
        robot.initializeServos();
        sensors = new DistanceSensor[4];
        if (limitS) {
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
        side = s;
    }

    double isEnabled(double val) {
        if (val > 0) return 1;
        else return 0;
    }

    void turnToAngle(double dest) {
        double thresh = 1;
        double currentAng = getAngle();

        while (currentAng < dest + thresh || currentAng > dest - thresh) {
            double speed = (-1.6 / ((Math.abs(currentAng - dest)) + 3)) + 0.45;
            if (speed < 0.05) {
                break;
            }
            telemetry.addData("Angle", currentAng);
            telemetry.addData("Speed", speed);
            telemetry.update();
            if (dest - currentAng > currentAng - dest) {
                mecanums.move(0, 0, speed);
            } else {
                mecanums.move(0, 0, -speed);
            }
            currentAng = getAngle();
        }
        mecanums.stopNow();
    }

    void turnToAngle(double dest, double power) {
        double thresh = 1.5;
        double currentAng = getAngle();

        while (currentAng < dest + thresh || currentAng > dest - thresh) {
            double speed = (-1.6 / ((Math.abs(currentAng - dest)) + 3)) + 0.45;
            if (speed < 0.05) {
                break;
            }
            telemetry.addData("Angle", currentAng);
            telemetry.addData("Speed", speed);
            telemetry.update();
            if (dest - currentAng > currentAng - dest) {
                mecanums.move(90, power, speed);
            } else {
                mecanums.move(90, power, -speed);
            }
            currentAng = getAngle();
        }
        mecanums.stopNow();
    }

    //void rotateAndPos(double angle, double direction){}
    void moveWithEncoder(double Xdest, double Ydest, boolean YthenX) {
        double Xdiff = Math.abs(Math.abs(Xdest + 72) - Math.abs(X + 72));
        double Ydiff = Math.abs(Ydest - Y);
        double XTicks = (Xdiff * COUNTS_ROTATION) / (Math.PI * WHEEL_DIAMETER);
        double YTicks = (Ydiff * COUNTS_ROTATION) / (Math.PI * WHEEL_DIAMETER);


        encoders.resetEncoders();
        double speed = 0.65;

        double ticks = encoders.getTicks();
        double Xdir = 0;
        double Ydir = 90;

        if (Xdest > X) {
            Xdir = 180;
        }
        if (Ydest < Y) {
            Ydir = 270;
        }
        if (side == Side.RED) {
            Xdir = Math.abs(Xdir - 180);
        }

        if (YthenX) {
            ticks = encoders.getTicks();
            while (opModeIsActive() && ticks < YTicks) {
                ticks = encoders.getTicks();
                mecanums.absMove(Ydir, speed, getAngle());
            }
            encoders.resetEncoders();
            ticks = encoders.getTicks();
            while (opModeIsActive() && ticks < XTicks) {
                ticks = encoders.getTicks();
                mecanums.absMove(Xdir, speed, getAngle());
            }
        } else {
            while (opModeIsActive() && ticks < XTicks) {
                ticks = encoders.getTicks();
                mecanums.absMove(Xdir, speed, getAngle());
            }
            encoders.resetEncoders();

            ticks = encoders.getInches();
            while (opModeIsActive() && ticks < YTicks) {
                ticks = encoders.getTicks();
                mecanums.absMove(Ydir, speed, getAngle());
            }
        }
        mecanums.stopNow();
    }

    void moveWithEncoderTurn(double Xdest, double Ydest, double wantedAngle, double initAngle) {
        double currAngle = getAngle();
        double thresh = 1.5;
        double turn;
        double Xdiff = Math.abs(Math.abs(Xdest + 72) - Math.abs(X + 72));
        double Ydiff = Math.abs(Ydest - Y);
        double XTicks = (Xdiff * COUNTS_ROTATION) / (Math.PI * WHEEL_DIAMETER);
        double YTicks = (Ydiff * COUNTS_ROTATION) / (Math.PI * WHEEL_DIAMETER);


        encoders.resetEncoders();
        double speed = 0.65;

        double ticks;
        double Xdir = 0;
        double Ydir = 90;

        if (Xdest > X) {
            Xdir = 180;
        }
        if (Ydest < Y) {
            Ydir = 270;
        }
        if (side == Side.RED) {
            Xdir = Math.abs(Xdir - 180);
        }
        ticks = encoders.getTicks();
        while (opModeIsActive() && ticks < YTicks) {
            currAngle = getAngle();
            if (currAngle < wantedAngle - thresh || currAngle > wantedAngle + thresh) {
                turn = 0.3;
            } else {
                turn = 0;
                updatePosition();
                Xdiff = Math.abs(Math.abs(Xdest + 72) - Math.abs(X + 72));
                Ydiff = Math.abs(Ydest - Y);

                XTicks = (Xdiff * COUNTS_ROTATION) / (Math.PI * WHEEL_DIAMETER);
                YTicks = (Ydiff * COUNTS_ROTATION) / (Math.PI * WHEEL_DIAMETER);
            }
            ticks = encoders.getTicks();
            mecanums.absMoveTurn(Ydir, speed, initAngle - getAngle(), turn);
        }

        encoders.resetEncoders();

        ticks = encoders.getTicks();

        while (opModeIsActive() && ticks < XTicks) {
            if (currAngle < wantedAngle - thresh || currAngle > wantedAngle + thresh) {
                turn = 0.3;
            } else {
                turn = 0;
                updatePosition();
            }
            ticks = encoders.getTicks();
            mecanums.absMoveTurn(Xdir, speed, initAngle - getAngle(), turn);
        }

        mecanums.stopNow();
    }

    public void moveToClosePoint(double xDest, double yDest, double turnAngle){
        //Rotate robot left increases angle
        updatePosition();
        double xDiff = Math.abs(X - xDest);
        double yDiff = Math.abs(Y - yDest);
        double thresh = 1.5;
        double angle;
        double turn;

        angle = Math.atan2(yDiff, xDiff);

        if(getAngle() > turnAngle + thresh){
            turn = -0.4;
        }
        else if(getAngle() < turnAngle -thresh){
            turn = 0.4;
        }
        else{
            turn = 0;
        }

        while(turn != 0){
            telemetry.addData("angle: ", angle);
            telemetry.update();
            if(getAngle() > turnAngle + thresh){
                turn = -0.4;
            }
            else if(getAngle() < turnAngle -thresh){
                turn = 0.4;
            }
            else{
                turn = 0;
            }

            mecanums.absMoveTurn(angle, 0.4, getAngle(), turn);
            sleep(40);
            mecanums.stopNow();

        }
        updatePosition();
    }
    double getAvgDis(int index){
        double iterations = 3;
        double mean = 0;
        for(int a = 0; a < iterations; a++){
            mean += sensors[index].getDistance(DistanceUnit.INCH);
        }
        return mean/iterations;
    }


   /* public void moveWithLazer(double Xdest, double Ydest){
        double counter = 0;
        double angle = getAngle();
        double thresh = 1.2;
        angleQuad aQuad;
        Ydest = Ydest/2;
        double Xdiff = Math.abs(Math.abs(Xdest + 72) - Math.abs(X + 72));
        double Ydiff = Math.abs(Ydest - Y);
        double hypo = Math.sqrt(Math.pow(Math.abs(X) - Math.abs(Xdest), 2) + Math.pow(Math.abs(Y) - Math.abs(Ydest), 2));
        updateAveragePosition();

            double direction = Math.toDegrees(Math.atan((Ydiff) / Xdiff));

        if(X < Xdest){
            direction = 180 - direction;
        }
        if(Y > Ydest){
            if(X < Xdest){
                direction = 180 + direction;
            }
            else {
                direction = 270 - direction;
            }
        }
        double absX = Math.abs(X);
        while(opModeIsActive() && (absX < Math.abs(Xdest) - thresh || absX > Math.abs(Xdest) + thresh) && (Y < Ydest - thresh || Y > Ydest + thresh)){

            absX = Math.abs(X);
            telemetry.addData("Min: ",  (Math.abs(Xdest) - thresh) + " Max: " + (Math.abs(Xdest) + thresh));
            telemetry.addData("Min: ",  (Math.abs(Ydest) - thresh) + " Max: " +  (Math.abs(Ydest) + thresh));
            telemetry.addData("X",  absX);
            telemetry.addData("Y",  Y);
            telemetry.addData("Direction", direction);
            telemetry.addData("Update", getAngle());
            telemetry.addData("hypo ", hypo);
            telemetry.update();
            if(counter % 10 == 0) {
                updatePosition();
            }
            Xdiff = Math.abs(Math.abs(Xdest + 72) - Math.abs(X + 72));
            Ydiff = Math.abs(Math.abs(Ydest) - Math.abs(Y));
            direction = Math.toDegrees(Math.atan(Ydiff/Xdiff));
            hypo = Math.sqrt(Math.pow(Math.abs(X) - Math.abs(Xdest), 2) + Math.pow(Math.abs(Y) - Math.abs(Ydest), 2));
            //speed = ((-3/(2 * hypo)) + 0.5);
            //double speed = (hypo/30) + 0.1;
            //double speed = (Math.pow(hypo, 2)/80) + 0.1;
            double speed = 0.5;
            if(hypo < 15){
                speed = 0.35;
            }
            if(X < Xdest){
                direction = 180 - direction;
            }
            if(Y > Ydest){
                if(X < Xdest){
                    direction = 180 + direction;
                }
                else {
                    direction = 270 - direction;
                }
            }
            direction = direction + 90;
            angle = getAngle();
            mecanums.absMove(direction, speed, angle);

        }
        mecanums.stopNow();
        telemetry.clearAll();
        telemetry.addData("DONE", "");
        telemetry.update();
    }*/
}