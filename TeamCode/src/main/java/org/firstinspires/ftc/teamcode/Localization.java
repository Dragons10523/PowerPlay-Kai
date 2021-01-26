package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public abstract class Localization extends Control {

    public ElapsedTime time;
    protected double theta;
    protected boolean turningFlag = false;
    private double targetAngle;
    private int turnDirection;
    protected Geometry.Point robotLocation;
    protected Geometry.Point velocity;
    public Geometry geometry;
    private Lightsaber lightsaber;
    private double prevTime = 0;

    final double DELTA_T = 0.01;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String VUFORIA_KEY =
            "AQkCE3T/////AAABmacf2GUqlUiet5GB5KP6epRTyl96EqEA9gcG1VI99J81/l4NkkwX6Nx/L7BTIL+1Z3R7yorhZ4YW1N6InBS7l7o8rKNgpbwWzBkfh3Unneq6h5xeyhbILzENlxNOVSibrronjr5199YlL3+PbMazXySVa5mnY2hXXO9CXcuv/pfEyCblbkFchA3D+Ngpkpg8CSbpkXeM6aKgGEXsnBZO7xUtE8p71aFIew1Coez3KBM5n12hoov/SdKC3O6GAcbMTX3A9wVZgACfXmw4F4Skgm/QjcfG9dOH0w7Wj3Ne6haXCVS13A2uYecamReSZZyT+BatU5nfh9t4KjRtgKGf/SMAJLIoBcbdYxBqiTmyG3WN";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void startLocalization() {
        startLocalization(true);
    }

    public void startLocalization(boolean vuFlag) {
        if(vuFlag) {
            initVuforia();
            initTfod();
        }
        lightsaber = thalatte.lightsaber;
        geometry = thalatte.geometry;
        time = new ElapsedTime();
        lightsaber.estimate();
        robotLocation = lightsaber.position;
        velocity = geometry.point(0, 0);
    }

    public void updateLocalization() {
        double delta_t, delta_x, delta_y;
        theta = thalatte.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle;
        theta %= Math.PI * 2;
        lightsaber.setTheta(theta);
        if(turningFlag) updateTurnTo();
        lightsaber.estimate();
        delta_t = time.seconds() - prevTime;
        delta_x = lightsaber.position.x - robotLocation.x;
        delta_y = lightsaber.position.y - robotLocation.y;
        velocity = geometry.point(delta_x / delta_t, delta_y / delta_t);
        prevTime += delta_t;
        robotLocation = lightsaber.position;
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

        if(targetAngle - this.theta > 0) turnDirection =  1;
        if(targetAngle - this.theta < 0) turnDirection = -1;
        if(Math.abs(targetAngle - this.theta) > Math.PI) turnDirection = -turnDirection;

        double power = 1 * turnDirection;

        if(Math.abs(targetAngle - theta) > Math.PI / 8 ||
           Math.abs(targetAngle - theta) < Math.PI * 15 / 8) power *= 0.25;

        drive(-power,power);
    }

    public void stopTurnTo(){
        turningFlag = false;
        drive(0,0);
    }

    private Geometry.Point linearCurve(double t, Geometry.Point p1, Geometry.Point p2){
        return geometry.point(p1.x + t * (p2.x - p1.x), p1.y + t * (p2.y - p1.y));
    }

    public Geometry.Line bezierLine(double t, Geometry.Point ... points){
        Geometry.Point[] list = new Geometry.Point[points.length - 1];
        for(int i = 0; i < points.length - 1; i++){
            list[i] = linearCurve(t, points[i], points[i + 1]);
        }
        if(list.length == 2) return geometry.line(list[0], list[1]);
        return bezierLine(t, list);
    }

    public double velocity(double time, double speed, Geometry.Point ... points){
        double t1 = clamp(speed * (time - (DELTA_T / 2)), 0, 1);
        double t2 = clamp(speed * (time + (DELTA_T / 2)), 0, 1);

        Geometry.Line l     = bezierLine(speed * time, points);
        Geometry.Point p1   = linearCurve(t1, l.p1, l.p2     );
        Geometry.Point p2   = linearCurve(t2, l.p1, l.p2     );
        Geometry.Line delta = geometry.line(p1, p2  );

        return delta.getDistance() / DELTA_T;
    }

    public double rotVelocity(double time, double speed, Geometry.Point ... points){
        double t1 = clamp(speed * (time - (DELTA_T / 2)), 0, 1);
        double t2 = clamp(speed * (time + (DELTA_T / 2)), 0, 1);

        Geometry.Line l1 = bezierLine(t1, points);
        Geometry.Line l2 = bezierLine(t2, points);

        double delta_theta  = ((l2.theta - l1.theta) % (Math.PI * 2));
           if (delta_theta  >                           Math.PI)
               delta_theta -=                           Math.PI * 2;

        return delta_theta / DELTA_T;
    }

    public void driveWithTheta(double velocity, double rot_velocity){
        final double width = 1.0;
        double x, vel_r, vel_l;
        if(rot_velocity != 0) {
            x = velocity / (rot_velocity * Math.PI * 2);
            vel_r = (width + x) * rot_velocity;
            vel_l = (x - width) * rot_velocity;
        } else {
            vel_r = velocity;
            vel_l = velocity;
        }

        unstablyDrive(vel_l,vel_r); //drive(power(vel_l),power(vel_r));
    }

    public double power(double velocity){
        final double const1 = 14.5; // Maximum Battery?
        final double const2 = 96;   // Maximum Velocity?
        return clamp((thalatte.vs.getVoltage() / const1) * (velocity / const2), -1, 1);
    }

    public void unstablyPower(double velocity, DcMotor motor){
        double oldPower   = motor.getPower();
        double currentVel = this.velocity.distance();
        double deltaVel   = velocity - currentVel;
        double deltaPower = power(deltaVel);
        motor.setPower(clamp(oldPower + deltaPower,-1,1));
    }

    public void unstablyDrive(double lvel, double rvel){
        unstablyPower(lvel,thalatte.backLeft);
        unstablyPower(lvel,thalatte.frontLeft);
        unstablyPower(rvel,thalatte.backRight);
        unstablyPower(rvel,thalatte.frontRight);
    }

    public boolean bezierDrive(double time, double speed, Geometry.Point ... points) {
        if(time * speed > 1.0){
            zero();
            return false;
        }
        double velocity = velocity(time,speed,points);
        double rotVelocity = rotVelocity(time,speed,points);
        driveWithTheta(velocity,rotVelocity);
        return true;
    }

    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod() {
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
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if(updatedRecognitions == null) return 0;
        for(Recognition r : updatedRecognitions){
            if(r.equals("Quad"))   return 4;
            if(r.equals("Single")) return 1;
        }
        return 0;
    }

    public void stopTfodCrap(){
        tfod.shutdown();
        vuforia = null;
    }
}

// ☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭