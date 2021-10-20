package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
        if(theta_offset == null) theta_offset = (Double)(double)ahi.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle; // Bit of a mess to convert float -> double -> Double
    }

    public boolean driveDist(double dist) {
        int ticks = (int)(dist*CONVERSION_FACTOR);

        int leftTarget = ticks+ahi.leftA.getCurrentPosition();
        int rightTarget = ticks+ahi.rightA.getCurrentPosition();

        // Defining PID variables
        double kP = 1;
        double kI = 1;
        double kD = 1;

        double PLeft = leftTarget-ahi.leftA.getCurrentPosition();
        double PLeftPrev = PLeft;
        double ILeft = 0;
        double DLeft = 0;

        double PRight = rightTarget-ahi.rightA.getCurrentPosition();
        double PRightPrev = PRight;
        double IRight = 0;
        double DRight = 0;

        // Setting up timer for deltaTime
        ElapsedTime time = new ElapsedTime(); // Potentially change to a global timer

        double lastTime = time.milliseconds();
        double deltaTime = 0;

        double timeInBounds = 0;

        while(opModeIsActive() && timeInBounds > 300) {
            deltaTime = time.milliseconds()-lastTime;

            PLeft = leftTarget-ahi.leftA.getCurrentPosition();
            PRight = rightTarget-ahi.rightA.getCurrentPosition();

            ILeft += PLeft*deltaTime;
            IRight += PRight*deltaTime;

            try { // In case of divide by zero error
                DLeft = (PLeft - PLeftPrev) / deltaTime;
                DRight = (PRight - PRightPrev) / deltaTime;
            } catch (ArithmeticException e) {
                DLeft = 0;
                DRight = 0;
            }

            drive(kP*PLeft + kI*ILeft + kD*DLeft, kP*PRight + kI*IRight + kD*DRight);

            PRightPrev = PRight;
            PLeftPrev = PLeft;
            lastTime = time.milliseconds();

            if(Math.abs(PLeft) < 350 && Math.abs(PRight) < 350) {
                timeInBounds += deltaTime;
            } else {
                timeInBounds = 0;
            }
        }

        return false;
    }

    public ArmPosition getFieldOrientation() {
        // TODO: AprilTag fiducial tracker
        return ArmPosition.HIGH;
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
        drive(0,0);
        sleep(350);
        theta = collapseAngle(ahi.imu.getAngularOrientation().toAngleUnit(AngleUnit.RADIANS).firstAngle + (Math.PI / 2) - theta_offset);
        if(((theta >= targetAngle - 0.05)&&(theta <= targetAngle + 0.05))||((theta - (Math.PI * 2) >= targetAngle - 0.05)&&(theta - (Math.PI * 2) <= targetAngle + 0.05)))
            turningFlag = false;
    }
}
