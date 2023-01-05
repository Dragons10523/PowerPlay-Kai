package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.processors.AutoControl;
import org.firstinspires.ftc.teamcode.processors.VecUtils;

@Autonomous(name = "AutoMoveTest", group = "Test")
public class AutoMoveTest extends AutoControl {
    @Override
    public void start() {
        kai.deadwheels.setTransform(0, 0, 0);

        /*double spinSpeed;
        do {
            if(isStopRequested) break;
            kai.deadwheels.wheelLoop();

            telemetry.addData("X", kai.deadwheels.currentX);
            telemetry.addData("Y", kai.deadwheels.currentY);
            telemetry.addData("R", kai.deadwheels.currentAngle);
            telemetry.update();

            spinSpeed = 5 * VecUtils.TAU - kai.deadwheels.currentAngle;
            mecanumDrive(0, 0, -spinSpeed, DriveMode.GLOBAL);
        } while(Math.abs(spinSpeed) > 0.01);*/

        /*moveToTile(1, 1);
        moveToTile(-1, 0);
        moveToTile(-1, 2);*/

        moveToTile(3, 0);
        moveToTile(0, 0);

        sleep(1000);
    }
}
