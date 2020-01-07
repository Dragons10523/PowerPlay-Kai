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

        moveWithEncoder(50, 35);

        updatePosition();

        sleep(1000);

        turnToAngle(180, 0.4);
        sleep(500);

        turnToAngle(90, 0.35);

        updatePosition();

        sleep(200);

        telemetry.addData("X Y", X + "   " + Y);
        telemetry.update();

        moveWithEncoder(0, 39);

    }
}
