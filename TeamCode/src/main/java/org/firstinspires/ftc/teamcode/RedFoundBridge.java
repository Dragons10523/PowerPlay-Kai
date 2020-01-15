package org.firstinspires.ftc.teamcode;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RedFoundBridge")
public class RedFoundBridge extends Localization {
    @Override
    public void runOpMode(){
        init(hardwareMap, false, Side.RED);
        waitForStart();

        mecanums.stopNow();

        updatePosition();

        telemetry.addData("X Y", X + " " + Y);
        telemetry.update();

        sleep(250);

        moveWithEncoder(55, 30, false);

        rightClaw.setPosition(0.85);
        leftClaw.setPosition(0.75);

        sleep(1500);

        updatePosition();
        moveWithEncoder(X, 40, false);
        sleep(250);
        turnToAngle(0, 0.4);
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
        sleep(250);

        updatePosition();

        sleep(200);

        telemetry.addData("X Y", X + "   " + Y);
        telemetry.update();
        sleep(500);
        moveWithEncoder(4, 40   , true);

    }
}
