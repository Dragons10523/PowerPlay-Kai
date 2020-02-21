package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    @TeleOp(name = "MoveW/lazer")
public class SwipeAutoBlue extends Localization {

    public void runOpMode() {
        double xDiff;
        double yDiff;
        double angle;
        double Xdest = -48;
        double Ydest = 36;
        double turn = 0;
        double thresh  = 1;
        double angleThresh = 3;
        init(hardwareMap, false, Side.BLUE);
        double getAng = getAngle();
        waitForStart();
        updatePosition();
        //moveToClosePoint(-48,24,270);
        xDiff =  Math.abs(Math.abs(Xdest + 72) - Math.abs(X + 72));
        yDiff = Math.abs(Y) - Math.abs(Ydest);
        angle = Math.atan2(yDiff, xDiff);
        turnToAngle(turn);
        while(opModeIsActive() && (Math.abs(X) < Math.abs(Xdest) - thresh || Math.abs(X) > Math.abs(Xdest) + thresh) && (Y < Ydest - thresh || Y > Ydest + thresh)){
            getAng = getAngle();
            updatePosition();
            xDiff =  Math.abs(Math.abs(Xdest + 72) - Math.abs(X + 72));
            yDiff = Math.abs(Y) - Math.abs(Ydest);
            angle = Math.atan2(yDiff, xDiff);
            telemetry.addData("xDiff: ", xDiff);
            telemetry.addData("yDiff: ", yDiff);
            telemetry.addData("angle: ", angle);
            telemetry.addData("Gyro", getAng);
            telemetry.update();
            mecanums.absMove(Math.toDegrees(angle), 0.4, getAng);
        }
        mecanums.stopNow();

        /*
        init(hardwareMap, false, Side.BLUE);
        waitForStart();
        updatePosition();
        while(opModeIsActive()) {
            mecanums.absMove(90, 0.4, getAngle());
        }
        */
    }
}
