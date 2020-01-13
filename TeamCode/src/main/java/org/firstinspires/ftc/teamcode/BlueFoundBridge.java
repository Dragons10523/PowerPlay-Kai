package org.firstinspires.ftc.teamcode;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueFoundBridge")
public class BlueFoundBridge extends NewLocalization {
    @Override
    public void runOpMode(){
        init(hardwareMap, false, Side.BLUE);
        waitForStart();

        mecanums.stopNow();

        updatePosition(1);

        telemetry.addData("X Y", X + " " + Y);
        telemetry.update();

        sleep(250);

        moveWithEncoder(55, 28, false);

        rightClaw.setPosition(0.85);
        leftClaw.setPosition(0.75);

        sleep(1500);

        updatePosition();
        moveWithEncoder(X, 40, false);
        sleep(250);
        turnToAngle(180, 0.4);
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
        sleep(250);

        updatePosition();

        sleep(200);

        telemetry.addData("X Y", X + "   " + Y);
        telemetry.update();
        sleep(500);
        moveWithEncoder(3, 35, true);

    }
}
