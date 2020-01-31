package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turn and move")
public class DriveAndTurn extends Localization {
    double thresh = 6;
    @Override
    public void runOpMode() {
        init(hardwareMap, false, Side.BLUE);
        waitForStart();
        while(opModeIsActive()) {
            while(getAngle() > 0 + thresh){
                mecanums.absMoveTurn(270, 0.26, getAngle(), -0.26);
                telemetry.addData("Angle", getAngle());
                telemetry.update();
            }
            mecanums.stopNow();
        }
    }
}
