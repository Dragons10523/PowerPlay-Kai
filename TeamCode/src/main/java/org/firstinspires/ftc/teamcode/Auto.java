package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto")
public class Auto extends Localization{

    enum Stage {
        DETECT,
        WOBBLEGOAL,
        RINGS,
        POWERSHOTS,
        HIGHGOALS,
        ZERO
    }

    enum Alliance {
        BLUE,
        RED
    }

    double time1;
    double currentTime;
    double currentSpeed;

    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        waitForStart();
        Stage stage = Stage.DETECT;
        Alliance alliance = Alliance.RED;
        boolean stageComplete = false;

        Geometry.Point wobbleA        = null;
        Geometry.Point wobbleB        = null;
        Geometry.Point wobbleC        = null;
        Geometry.Point ps1            = null;
        Geometry.Point ps2            = null;
        Geometry.Point ps3            = null;
        Geometry.Point highGoal       = null;
        Geometry.Point rings          = null;
        Geometry.Point[] ringPoints   = new Geometry.Point[4];
        Geometry.Point[] wobblePoints = new Geometry.Point[4];

        int highGoalShots  = 0;
        boolean turnedFlag = false;
        startLocalization();
        while(opModeIsActive()){
            updateLocalization();
            switch(stage){
                case DETECT:
                    //TODO OpenCV crap
                    alliance = Alliance.RED;
                    if(alliance == null){
                        alliance = Alliance.RED;
                    }
                    if(alliance == Alliance.RED){
                        wobbleA  = thalatte.geometry.point(72,  132);
                        wobbleB  = thalatte.geometry.point(120, 108);
                        wobbleC  = thalatte.geometry.point(132, 132);
                        ps1      = thalatte.geometry.point(88,  144);
                        ps2      = thalatte.geometry.point(96,  144);
                        ps3      = thalatte.geometry.point(104, 144);
                        highGoal = thalatte.geometry.point(108, 144);
                        rings    = thalatte.geometry.point(108,  48);
                        ringPoints[0] = robotLocation;
                        ringPoints[1] = thalatte.geometry.point(0, 0);
                        ringPoints[2] = thalatte.geometry.point(0, 0);
                        ringPoints[3] = rings;
                        wobblePoints[0] = robotLocation;
                    } else if(alliance == Alliance.BLUE){
                        wobbleA  = thalatte.geometry.point(0,0);
                        wobbleB  = thalatte.geometry.point(0,0);
                        wobbleC  = thalatte.geometry.point(0,0);
                        ps1      = thalatte.geometry.point(0,0);
                        ps2      = thalatte.geometry.point(0,0);
                        ps3      = thalatte.geometry.point(0,0);
                        highGoal = thalatte.geometry.point(0,0);
                        rings    = thalatte.geometry.point(0,0);
                    }
                    highGoalShots = rings();
                    stopTfodCrap();
                    switch (highGoalShots){
                        case 0:
                            wobblePoints[1] = thalatte.geometry.point(0, 0);
                            wobblePoints[2] = thalatte.geometry.point(0, 0);
                            wobblePoints[3] = wobbleA;
                            break;
                        case 1:
                            wobblePoints[1] = thalatte.geometry.point(0, 0);
                            wobblePoints[2] = thalatte.geometry.point(0, 0);
                            wobblePoints[3] = wobbleB;
                            break;
                        case 4:
                            wobblePoints[1] = thalatte.geometry.point(0, 0);
                            wobblePoints[2] = thalatte.geometry.point(0, 0);
                            wobblePoints[3] = wobbleC;
                            break;
                    }
                    stage = Stage.POWERSHOTS;
                    break;
                case RINGS:
                    currentTime = time.seconds() - time1;
                    currentSpeed = 1.0;

                    if(!bezierDrive(currentTime, currentSpeed, ringPoints)){
                        stage = Stage.HIGHGOALS;
                        time1 = time.seconds();
                    }
                    break;
                case WOBBLEGOAL:
                    currentTime = time.seconds() - time1;
                    currentSpeed = 1.0;

                    if(!bezierDrive(currentTime, currentSpeed, wobblePoints)){
                        stage = Stage.ZERO;
                        time1 = time.seconds();
                    }
                    break;
                case POWERSHOTS:
                    ps = PowerShot.LEFT;
                    Geometry.Line line = thalatte.geometry.line(robotLocation,ps1);
                    startTurnTo(line.theta);
                    if(!turningFlag){
                        shootDistance(Math.sqrt((robotLocation.x + ps1.x)*(robotLocation.x + ps1.x)+(robotLocation.y + ps1.y)*(robotLocation.y + ps1.y)));
                        ps = PowerShot.CENTER;
                    }
                    line = thalatte.geometry.line(robotLocation,ps2);
                    startTurnTo(line.theta);
                    if(!turningFlag){
                        shootDistance(Math.sqrt((robotLocation.x + ps2.x)*(robotLocation.x + ps2.x)+(robotLocation.y + ps2.y)*(robotLocation.y + ps2.y)));
                        ps = PowerShot.RIGHT;
                    }
                    line = thalatte.geometry.line(robotLocation,ps3);
                    startTurnTo(line.theta);
                    if(!turningFlag){
                        shootDistance(Math.sqrt((robotLocation.x + ps3.x)*(robotLocation.x + ps3.x)+(robotLocation.y + ps3.y)*(robotLocation.y + ps3.y)));
                        stage = Stage.RINGS;
                        time1 = time.seconds();
                    }
                    break;
                case HIGHGOALS:
                    if(!turningFlag && !turnedFlag) startTurnTo(thalatte.geometry.line(robotLocation, highGoal).theta);
                    if(turningFlag) turnedFlag = true;
                    if(!turningFlag && turnedFlag) {
                        shootDistance(thalatte.geometry.line(robotLocation, highGoal).getDistance());
                        highGoalShots -= 1;
                        turnedFlag = false;
                    }
                    if(highGoalShots <= 0) {
                        stage = Stage.WOBBLEGOAL;
                    }
                    break;
                case ZERO:
                    zero();
                    break;
            }
        }
    }
}

// ☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭