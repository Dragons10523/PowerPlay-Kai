package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AbstractAutonomous;

@Autonomous(name = "Drive Distance Test", group = "Testing")
public class DriveDistanceTest extends AbstractAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeValues();
        waitForStart();
        driveDist(12);
    }
}
