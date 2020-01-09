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

        sleep(1000);

        moveWithEncoder(50, 30);
        rightClaw.setPosition(0.85);
        leftClaw.setPosition(0.75);


        updatePosition();
        telemetry.addData("X Y", X + " " + Y);
        telemetry.update();
        sleep(2500);

        turnToAngle(180, 0.4);
        rightClaw.setPosition(0.85);
        leftClaw.setPosition(0.75);
        sleep(1000);

        turnToAngle(90);

        updatePosition();

        sleep(200);

        telemetry.addData("X Y", X + "   " + Y);
        telemetry.update();

        moveWithEncoder(0, 55);

    }
}
