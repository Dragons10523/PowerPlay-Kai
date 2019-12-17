package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public class Localization extends LinearOpMode {
    //list of all the corners
    public enum Corner {
        BlueDepot, RedDepot, BlueTri, RedTri
    }

    //Vuforia parameters
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    public static final boolean PHONE_IS_PORTRAIT = false;
    public static final float mmPerInch = 25.4f;
    public static final float mmTargetHeight = (6) * mmPerInch;
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField = 36 * mmPerInch;
    public OpenGLMatrix lastLocation = null;
    public float phoneXRotate = 0;
    public boolean targetVisible = false;
    public List<VuforiaTrackable> allTrackables;
    public String currentTrackable = null;
    public VuforiaTrackables targetsSkyStone = null;

    //Laser distance sensors
    public static final double rightLaserThresh = 7;
    public static final double rearLaserThresh = 7;
    public static final double frontLaserThresh = 5;
    public static DistanceSensor front, right, left, rear, cameraDis;
    double frontDis, rightDis, leftDis, rearDis, cameraDisDis;
    //global variables for localization and mecanums
    public static double X = 0, Y = 0;
    public static double stoneX = 0, stoneY = 0;
    public static boolean stoneVisible = false;
    public Corner corner = Corner.RedDepot;
    public OOPO funcs;
    public Encoders encode;
    public boolean XVisible = true;
    public boolean YVisible = true;
    public ProtoConfig bahumut;
    //Gyroscope variables
    public BNO055IMU imu;
    public double angleError = 0;
    BNO055IMU.Parameters gyrometers;
    //Global variables for speed and error
    public static double speed = 0.75;
    public double thresh = 1.5;

    public Servo leftClaw, rightClaw;

    @Override
    public void runOpMode() {
        initialize(hardwareMap);
        waitForStart();

        while (!isStopRequested()) {
            updatePosition();
            telemetry.addData("Target Visibility", targetVisible);
            telemetry.addData("Pos (in)", "{X, Y} = %.1f, %.1f", X, Y);
            if (stoneVisible) {
                telemetry.addData("Stone", stoneVisible);
                telemetry.addData("Stone Target", "{X, Y}: %.1f, %.1f", stoneX, stoneY);
            }
            telemetry.update();
        }
        targetsSkyStone.deactivate();
    }

    public void moveWithEncoder(double Xdest, double Ydest) {
        double Ydiff = Math.abs(Ydest - Y);
        double Xdiff = Math.abs(Xdest - X);
        encode.resetEncoders();
        double inches = encode.getInches();
        if (Xdest > X) {
            while (!isStopRequested() && inches < Xdiff) {
                inches = encode.getInches();
                funcs.absMove(180, speed, getAngle());
            }
            funcs.stopNow();
        } else {
            while (!isStopRequested() && inches < Xdiff) {
                inches = encode.getInches();
                funcs.absMove(0, speed, getAngle());
            }
            funcs.stopNow();
        }

        encode.resetEncoders();
        inches = encode.getInches();
        if (Ydest > Y) {
            while (!isStopRequested() && inches < Ydiff) {
                inches = encode.getInches();
                funcs.absMove(90, speed, getAngle());
            }
            funcs.stopNow();
        } else {
            while (!isStopRequested() && inches < Ydiff) {
                inches = encode.getInches();
                funcs.absMove(270, speed, getAngle());
            }
            funcs.stopNow();
        }

    }



    /*public void gotoCoordinates(double Xdest, double Ydest, boolean turn) {
        //the fucking finale :O
        updatePosition();
        if (turn) {
            findRotationY();
        }

        while ((!(Y > Ydest - thresh && Y < Ydest + thresh) || !YVisible) && !isStopRequested()) {
            updatePosition();
            telemetry.addData("Pos (in)", "{X, Y} = %.1f, %.1f", X, Y);
            telemetry.update();
            gotoCordY(Y, Ydest);
        }
        funcs.stopNow();
        updatePosition();
        if (turn) {
            findRotationX();
        }
        while ((!(X > Xdest - thresh && X < Xdest + thresh) || !XVisible) && !isStopRequested()) {
            updatePosition();
            telemetry.addData("Pos (in)", "{X, Y} = %.1f, %.1f", X, Y);
            telemetry.update();
            gotoCordX(X, Xdest);
        }
        funcs.stopNow();
    }*/

    public void turnToAngle(double dest, boolean left) {
        double angle = getAngle();
        double thresha = 0.5;
        if (angle < 0) {
            angle = 360 - Math.abs(angle);
        }

        while (angle < dest - thresha || angle > dest + thresha) {
            angle = getAngle();

            if (angle < 0) {
                angle = 360 - Math.abs(angle);
            }
            if (!left) {
                funcs.move(0, 0, -0.35);
            } else {
                funcs.move(0, 0, 0.35);
            }
        }

        funcs.stopNow();
    }

    public void gotoCordX(double X, double Xdest) {
        if (X > Xdest + thresh) {
            funcs.absMove(180, speed, getAngle());
        } else if (X < Xdest - thresh) {
            funcs.absMove(0, speed, getAngle());
        }
    }

    public void gotoCordY(double Y, double Ydest) {
        if (Y < Ydest + thresh) {
            funcs.absMove(270, speed, getAngle());
        } else if (Y > Ydest - thresh) {
            funcs.absMove(90, speed, getAngle());
        }
    }

    public double getAngle() {
        return imu.getAngularOrientation(INTRINSIC, ZYX, DEGREES).firstAngle + angleError;
    }

    public void updatePosition() {
        frontDis = front.getDistance(DistanceUnit.INCH) + frontLaserThresh;
        rightDis = right.getDistance(DistanceUnit.INCH) + rightLaserThresh;
        leftDis = left.getDistance(DistanceUnit.INCH) + rightLaserThresh;
        rearDis = rear.getDistance(DistanceUnit.INCH) + rearLaserThresh;
        cameraDisDis = cameraDis.getDistance(DistanceUnit.INCH);
        double trueFront = frontDis;
        double trueRight = rightDis;
        double trueLeft = leftDis;
        double trueRear = rearDis;
        final double Xtemp = X;
        final double Ytemp = Y;
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        stoneVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                if (trackable.getName().equals("Stone Target")) {
                    OpenGLMatrix stoneLocation = ((VuforiaTrackableDefaultListener) trackable.getListener()).getVuforiaCameraFromTarget();
                    if (stoneLocation != null) {
                        VectorF trans = stoneLocation.getTranslation();
                        stoneX = trans.get(0);
                        stoneY = trans.get(1);
                    }
                    stoneVisible = true;
                }
            }
        }

        boolean[] enabled = {true, true, true, true};
        double rawangle = getAngle();

        if (rawangle > -95 && rawangle < -85) {
            trueFront = leftDis;
            trueRight = frontDis;
            trueLeft = leftDis;
            trueRear = rearDis;
        } else if (rawangle > 85 && rawangle < 95) {
            trueFront = rearDis;
            trueRight = leftDis;
            trueLeft = frontDis;
            trueRear = leftDis;
        } else if (Math.abs(rawangle) > 175) {
            trueFront = leftDis;
            trueRight = leftDis;
            trueLeft = rearDis;
            trueRear = frontDis;
        }

        double[] values = {trueFront, trueRear, trueRight, trueLeft};

        for (int i = 0; i < 4; i++) {
            if (values[i] > 300) {

                enabled[i] = false;
            }
        }

        if (enabled[2] && enabled[0]) {
            corner = Corner.BlueDepot;
            Localization.X = -72 + values[2];
            Localization.Y = -72 + values[0];
        } else if ((enabled[3] && enabled[1])) {
            corner = Corner.BlueTri;
            Localization.X = 72 - values[3];
            Localization.Y = 72 - values[1];
        } else if ((enabled[3] && enabled[0])) {
            corner = Corner.RedTri;
            Localization.X = 72 - values[3];
            Localization.Y = -72 + values[0];
        } else if ((enabled[2] && enabled[1]) && !isStopRequested()) {
            corner = Corner.RedDepot;
            Localization.X = -72 + values[2];
            Localization.Y = 72 - values[1];
        }else {
            for (int i = 0; i < 5; i++) {
                if (enabled[i]) {
                    if (i == 1 || i == 2) {
                        Y = 72 - (values[1] + values[2]) / 2;
                        YVisible = true;
                        XVisible = false;
                        break;
                    } else if (i == 0) {
                        Y = -72 + values[0];
                        YVisible = true;
                        XVisible = false;
                        break;
                    } else if (i == 3) {
                        X = -72 + values[3];
                        XVisible = true;
                        YVisible = false;
                        break;
                    } else {
                        X = 72 - values[4];
                        XVisible = true;
                        YVisible = false;
                        break;
                    }
                }
            }
        }

    if(Math.abs(X)>70)

    {
        X = Xtemp;
    }
    if(Math.abs(Y)>70)

    {
        Y = Ytemp;
    }
}
    /*public void findRotationY() {
        switch (corner) {
            case RedDepot:
            case BlueDepot:
                turnToAngle(0);
                break;
            case RedTri:
            case BlueTri:
                turnToAngle(180);
                break;
        }
    }

    public void findRotationX() {
        switch (corner) {
            case RedDepot:
            case BlueTri:
                turnToAngle(270);
                break;
            case BlueDepot:
            case RedTri:
                turnToAngle(90);
                break;
        }
    }*/

    public void initialize(HardwareMap hwmap){
        float phoneYRotate;
        float phoneZRotate = 0;
        VuforiaLocalizer vuforia;


        bahumut = new ProtoConfig();
        bahumut.init(hwmap);
        funcs = new OOPO(bahumut.frontLeft, bahumut.frontRight, bahumut.rearRight, bahumut.rearLeft);
        encode = new Encoders(bahumut.frontRight, bahumut.frontLeft, bahumut.rearRight, bahumut.rearLeft);
        encode.resetEncoders();
        cameraDis = bahumut.cameraDis;
        front = bahumut.front;
        right = bahumut.right;
        left = bahumut.left;
        rear = bahumut.rear;
        leftClaw = bahumut.leftClaw;
        rightClaw = bahumut.rightClaw;

         VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AQkCE3T/////AAABmacf2GUqlUiet5GB5KP6epRTyl96EqEA9gcG1VI99J81/l4NkkwX6Nx/L7BTIL+1Z3R7yorhZ4YW1N6InBS7l7o8rKNgpbwWzBkfh3Unneq6h5xeyhbILzENlxNOVSibrronjr5199YlL3+PbMazXySVa5mnY2hXXO9CXcuv/pfEyCblbkFchA3D+Ngpkpg8CSbpkXeM6aKgGEXsnBZO7xUtE8p71aFIew1Coez3KBM5n12hoov/SdKC3O6GAcbMTX3A9wVZgACfXmw4F4Skgm/QjcfG9dOH0w7Wj3Ne6haXCVS13A2uYecamReSZZyT+BatU5nfh9t4KjRtgKGf/SMAJLIoBcbdYxBqiTmyG3WN";


        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        targetsSkyStone = vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");//c3po
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");//house wife robot
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");//blue humanoid robot
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");//r2d2 and bb8
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");//just bb8
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");//gay flat head robot
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");//cartoon network robot
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");//dj robot

        allTrackables = new ArrayList<>();
        allTrackables.addAll(targetsSkyStone);


        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -180;
        } else {
            phoneYRotate = 90;
        }
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        final float CAMERA_FORWARD_DISPLACEMENT = 0f * mmPerInch;
        final float CAMERA_VERTICAL_DISPLACEMENT = 2.75f * mmPerInch;
        final float CAMERA_LEFT_DISPLACEMENT = -8.5f;

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        targetsSkyStone.activate();
        gyrometers = new BNO055IMU.Parameters();
        gyrometers.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyrometers.calibrationDataFile = "BNO055IMUCalibration.json";
        gyrometers.loggingEnabled = true;
        gyrometers.loggingTag = "IMU";
        imu = hwmap.get(BNO055IMU.class, "imu");
        imu.initialize(gyrometers);
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

        sleep(100); //Changing modes requires a delay before doing anything else

        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);

        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);

        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        sleep(100); //Changing modes again requires a delay
        angleError = -imu.getAngularOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
    }

}

