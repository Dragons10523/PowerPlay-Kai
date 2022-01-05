package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* CLASS SUMMARY:
 * Contains basic functions used throughout autonomous, but which are not necessary in drive
 * */

public abstract class AbstractAutonomous extends Control {
    protected double theta = Math.PI / 2;
    private int turnDirection;
    protected double targetAngle;
    protected boolean turningFlag = false;
    public Double theta_offset = null; // Leave this as null, it is initialized in initializeValues

    public void initializeValues() {
        initialize();
        if(theta_offset == null) theta_offset = (Double)(double)ahi.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle; // Bit of a mess to convert float -> double -> Double
    }

    public boolean driveDist(double dist) {
        int ticks = (int)(dist*CONVERSION_FACTOR);

        ahi.leftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ahi.leftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ahi.rightA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ahi.rightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ahi.leftA.setTargetPosition(ticks);
        ahi.leftB.setTargetPosition(ticks);
        ahi.rightA.setTargetPosition(ticks);
        ahi.rightB.setTargetPosition(ticks);

        ahi.leftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ahi.leftB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ahi.rightA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ahi.rightB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(0.5, 0.5);

        while(ahi.leftA.isBusy() && ahi.leftB.isBusy() && ahi.rightA.isBusy() && ahi.rightB.isBusy() && opModeIsActive()) {
            sleep(10);
            telemetry.addData("Left A", ahi.leftA.getCurrentPosition());
            telemetry.addData("Left B", ahi.leftB.getCurrentPosition());
            telemetry.addData("Right A", ahi.rightA.getCurrentPosition());
            telemetry.addData("Right B", ahi.leftB.getCurrentPosition());
            telemetry.addData("Target", ticks);
            telemetry.update();
        }

        if(isStopRequested()) {
            zero();
            return true;
        }

        drive(0, 0);

        ahi.leftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ahi.leftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ahi.rightA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ahi.rightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        return false;
    }

    // Everything from this point on is copy pasted code from last year
    public static double collapseAngle(double theta) {
        return (((theta % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI));
    }

    public void startTurnTo(double theta) {
        turnDirection = 1;
        theta = collapseAngle(theta);

        if(theta - this.theta > 0) turnDirection =  1;
        if(theta - this.theta < 0) turnDirection = -1;
        if(Math.abs(theta - this.theta) > Math.PI) turnDirection = -turnDirection;

        targetAngle = theta;
        turningFlag = true;
    }

    public void updateTurnTo() {
        theta = collapseAngle(ahi.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle + (Math.PI / 2) - theta_offset);

        if(((theta >= targetAngle - 0.05)&&(theta <= targetAngle + 0.05)) || ((theta - (Math.PI * 2) >= targetAngle - 0.05)&&(theta - (Math.PI * 2) <= targetAngle + 0.05)) || isStopRequested()) {
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

    public void stopTurnTo() {
        if(isStopRequested()) {
            turningFlag = false;
            return;
        }
        drive(0,0);
        sleep(350);
        theta = collapseAngle(ahi.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle + (Math.PI / 2) - theta_offset);
        if(((theta >= targetAngle - 0.05)&&(theta <= targetAngle + 0.05))||((theta - (Math.PI * 2) >= targetAngle - 0.05)&&(theta - (Math.PI * 2) <= targetAngle + 0.05)) || isStopRequested())
            turningFlag = false;
    }
}
