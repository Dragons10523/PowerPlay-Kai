package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

public abstract class VuforiaAbstract extends Control {

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;


    private static final String VUFORIA_KEY =
            "AQkCE3T/////AAABmacf2GUqlUiet5GB5KP6epRTyl96EqEA9gcG1VI99J81/l4NkkwX6Nx/L7BTIL+1Z3R7yorhZ4YW1N6InBS7l7o8rKNgpbwWzBkfh3Unneq6h5xeyhbILzENlxNOVSibrronjr5199YlL3+PbMazXySVa5mnY2hXXO9CXcuv/pfEyCblbkFchA3D+Ngpkpg8CSbpkXeM6aKgGEXsnBZO7xUtE8p71aFIew1Coez3KBM5n12hoov/SdKC3O6GAcbMTX3A9wVZgACfXmw4F4Skgm/QjcfG9dOH0w7Wj3Ne6haXCVS13A2uYecamReSZZyT+BatU5nfh9t4KjRtgKGf/SMAJLIoBcbdYxBqiTmyG3WN";

    private static final float mmPerInch      = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;

    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia  = null;

    boolean isInited = false;

    WebcamName webcamName = null;

    boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    VuforiaTrackables targetsUltimateGoal = null;
    VuforiaTrackable blueTowerGoalTarget  = null;
    VuforiaTrackable redTowerGoalTarget   = null;
    VuforiaTrackable redAllianceTarget    = null;
    VuforiaTrackable blueAllianceTarget   = null;
    VuforiaTrackable frontWallTarget      = null;
    List<VuforiaTrackable> allTrackables  = null;

    VectorF translation  = null;
    Orientation rotation = null;
    Orientation rotation2 = null;

    public void initAVuforia(){
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        parameters.cameraName = webcamName;

        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");

        blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");

        redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");

        redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");

        blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");

        frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        allTrackables = new ArrayList<>(targetsUltimateGoal);

        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));
        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));


        phoneYRotate = -90;

        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 8.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
        targetsUltimateGoal.activate();
        isInited = true;
    }

    public void stopAVuforia(){
        targetsUltimateGoal.deactivate();
        lastLocation = null;
        vuforia      = null;

        webcamName = null;

        targetVisible   = false;
        phoneXRotate    = 0;
        phoneYRotate    = 0;
        phoneZRotate    = 0;

        targetsUltimateGoal = null;
        blueTowerGoalTarget = null;
        redTowerGoalTarget  = null;
        redAllianceTarget   = null;
        blueAllianceTarget  = null;
        frontWallTarget     = null;
        allTrackables       = null;
        translation         = null;
        rotation            = null;
        rotation2           = null;
    }

    public void updateAV() {
        if(!isStopRequested() && isInited) {
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    targetVisible = true;

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }
            if (targetVisible) {
                translation = lastLocation.getTranslation();
                rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                rotation2 = Orientation.getOrientation(lastLocation, INTRINSIC, XYZ, DEGREES);
            }
        }
    }
}