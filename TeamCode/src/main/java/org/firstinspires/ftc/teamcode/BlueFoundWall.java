package org.firstinspires.ftc.teamcode;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BlueFoundWall")
public class BlueFoundWall extends Localization {
    @Override
    public void runOpMode(){
        init(hardwareMap, false, Side.BLUE);
        waitForStart();

        mecanums.stopNow();

        updatePosition();

        moveWithEncoder(50, 28, false);

        rightClaw.setPosition(0.85);
        leftClaw.setPosition(0.75);
        sleep(1000);

        updatePosition();
        moveWithEncoder(X, 40, false);

        turnToAngle(180, 0.4);
        rightClaw.setPosition(0);
        leftClaw.setPosition(0);
        sleep(1000);
        updatePosition();

        moveWithEncoder(3, 65, false);

    }
}
