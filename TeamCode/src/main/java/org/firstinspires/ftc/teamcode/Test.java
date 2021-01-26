package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Test")
public class Test extends Localization {
    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        waitForStart();
        startLocalization();
        Double startTime = null;
        Geometry.Point[] points = new Geometry.Point[4];
        points[0] = geometry.point(10,10); // 10, 10
        points[1] = geometry.point(60,10);
        points[2] = geometry.point(60,60);
        points[3] = geometry.point(10,60);
        startTurnTo(geometry.line(points[0],points[1]).theta);
        while(opModeIsActive()) {
            updateLocalization();
            if(turningFlag) continue;
            if(startTime == null) startTime = time.seconds();
            if(startTime - time.seconds() >= 1.0) continue;
            bezierDrive(time.seconds() - startTime, 1, points);
        }
        stop();
    }
}

// ☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭