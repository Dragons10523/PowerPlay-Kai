package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public abstract class Localization extends VuforiaAbstract {

    protected double theta = Math.PI / 2;
    public Double theta_offset = null;
    protected boolean turningFlag = false;
    protected boolean aimingFlag = false;
    protected double targetAngle;
    private int turnDirection;
    OpenCvCamera camera;
    CVinator cv;

    final double DELTA_T = 0.01;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String VUFORIA_KEY =
            "AQkCE3T/////AAABmacf2GUqlUiet5GB5KP6epRTyl96EqEA9gcG1VI99J81/l4NkkwX6Nx/L7BTIL+1Z3R7yorhZ4YW1N6InBS7l7o8rKNgpbwWzBkfh3Unneq6h5xeyhbILzENlxNOVSibrronjr5199YlL3+PbMazXySVa5mnY2hXXO9CXcuv/pfEyCblbkFchA3D+Ngpkpg8CSbpkXeM6aKgGEXsnBZO7xUtE8p71aFIew1Coez3KBM5n12hoov/SdKC3O6GAcbMTX3A9wVZgACfXmw4F4Skgm/QjcfG9dOH0w7Wj3Ne6haXCVS13A2uYecamReSZZyT+BatU5nfh9t4KjRtgKGf/SMAJLIoBcbdYxBqiTmyG3WN";

    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    enum vuFlag {
        OFF,
        AUTO,
        AIM,
        CV
    }

    public static double collapseAngle(double theta){
        return (((theta % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI));
    }

    public void startLocalization() {
        startLocalization(vuFlag.OFF);
    }

    public void startLocalization(vuFlag vf) {
        sleep(3000);
        if(theta_offset == null) theta_offset = (Double)(double)thalatte.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle;
        if (vf == vuFlag.AUTO) {
            initVuforia();
            initTfod();
            sleep(3000);
        }else if(vf == vuFlag.AIM){
            initAVuforia();
        }else if(vf == vuFlag.CV){
            initCV();
        }
    }

    public void updateLocalization() {
        theta = collapseAngle(thalatte.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle + (Math.PI / 2) - theta_offset);
        if(turningFlag) updateTurnTo();
        else if(aimingFlag) autoAim();
        if(isInited) updateAV();
    }

    public void startTurnTo(double theta){
        turnDirection = 1;
        theta = collapseAngle(theta);

        if(theta - this.theta > 0) turnDirection =  1;
        if(theta - this.theta < 0) turnDirection = -1;
        if(Math.abs(theta - this.theta) > Math.PI) turnDirection = -turnDirection;

        targetAngle = theta;
        turningFlag = true;
    }

    public void updateTurnTo(){
        if(((theta >= targetAngle - 0.05)&&(theta <= targetAngle + 0.05))||((theta - (Math.PI * 2) >= targetAngle - 0.05)&&(theta - (Math.PI * 2) <= targetAngle + 0.05))) {
            stopTurnTo();
            return;
        }

        if(targetAngle - this.theta > 0) turnDirection =    1;
        if(targetAngle - this.theta < 0) turnDirection =   -1;
        if(Math.abs(targetAngle - this.theta) > Math.PI) turnDirection = -turnDirection;

        double power = Math.abs(targetAngle - this.theta);
        if(power > Math.PI)
            power = (2*Math.PI) - power;
        power *= 0.75 / Math.PI;
        power += 0.25;

        drive(-power*turnDirection,power*turnDirection);
    }

    public void stopTurnTo(){
        drive(0,0);
        sleep(350);
        theta = collapseAngle(thalatte.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle + (Math.PI / 2) - theta_offset);
        if(((theta >= targetAngle - 0.05)&&(theta <= targetAngle + 0.05))||((theta - (Math.PI * 2) >= targetAngle - 0.05)&&(theta - (Math.PI * 2) <= targetAngle + 0.05)))
            turningFlag = false;
    }

    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, "Quad", "Single");
        if (tfod != null) tfod.activate();
    }

    public int rings(){
        if(tfod == null) return 0;
        List<Recognition> recognitions = tfod.getUpdatedRecognitions();
        sleep(50); // We hate this
        if(recognitions == null) return 0;
        for(Recognition r : recognitions){
            if(r.getLabel().contains("Quad"))   return 4;
            if(r.getLabel().contains("Single")) return 1;
        }
        return 0;
    }

    public void stopTfodCrap(){
        tfod.shutdown();
        vuforia = null;
    }

    public void initCV(){
        cv = new CVinator();
        camera = OpenCvCameraFactory.getInstance().createWebcam(thalatte.webcam);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);
                camera.setPipeline(cv);
            }
        });
    }

    public void autoAim(){
        aimingFlag = true;
        final double target = 0.50;
        Double position = cv.averagePosition();
        if(position == null) return;
        double difference = position - target;
        if(Math.abs(difference) <= 0.05) {
            aimingFlag = false;
            return;
        }
        double power = difference * 2; // difference / 0.5
        power *= 0.75;
        power += 0.25;
        drive(-power,power);
    }
}

// ☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭