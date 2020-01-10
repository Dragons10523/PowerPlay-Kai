package org.firstinspires.ftc.teamcode;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueFound")
public class Autonomous extends NewLocalization {
    @Override
    public void runOpMode(){
        init(hardwareMap, false);
        waitForStart();

        mecanums.stopNow();

        updatePosition();

        telemetry.addData("X Y", X + " " + Y);
        telemetry.update();

        sleep(250);

        moveWithEncoder(55, 28);

        rightClaw.setPosition(0.85);
        leftClaw.setPosition(0.75);

        sleep(250);

        updatePosition();
        moveWithEncoder(X, 40);
        sleep(250);
        turnToAngle(180, 0.4);
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
        sleep(250);

        updatePosition();

        sleep(200);

        telemetry.addData("X Y", X + "   " + Y);
        telemetry.update();

        moveWithEncoder(5, 65);

    }
}
