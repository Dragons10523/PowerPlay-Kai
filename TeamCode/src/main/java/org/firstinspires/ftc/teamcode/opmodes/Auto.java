package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.processors.AutoControl;
import org.firstinspires.ftc.teamcode.processors.Control;
import org.firstinspires.ftc.teamcode.processors.SignalOpticalSystem;
import org.firstinspires.ftc.teamcode.processors.VecUtils;

public abstract class Auto extends AutoControl {
    public static final double PARK_TIME = 25;

    public void auto(FieldSide side) {
        resetRuntime();

        VectorF initialTransform;
        VectorF targetTile;
        double grabAngle;
        VectorF depotPosition;
        VectorF defaultPosition;
        VectorF alternatePositon;
        VectorF defaultTarget;
        VectorF alternateTarget;
        VectorF coneStackPosition;

        VectorF leftPark;
        VectorF middlePark;
        VectorF rightPark;

        switch(side) {
            case LEFT:
                initialTransform = new VectorF(1.13f, -(2f/24));
                targetTile = new VectorF(1, 2);
                grabAngle = -VecUtils.HALF_PI;
                depotPosition = new VectorF(0.45f, 2);
                defaultPosition = new VectorF(1.5f, 2);
                alternatePositon = new VectorF(2, 2);
                defaultTarget = new VectorF(48, 72);
                alternateTarget = new VectorF(72, 48);
                coneStackPosition = new VectorF(2, 60);

                leftPark = new VectorF(0, 2);
                middlePark = new VectorF(1, 2);
                rightPark = new VectorF(2, 2);
                break;
            case RIGHT:
            default:
                initialTransform = new VectorF(4.13f, -(2f/24));
                targetTile = new VectorF(4, 2);
                grabAngle = VecUtils.HALF_PI;
                depotPosition = new VectorF(4.55f, 2);
                defaultPosition = new VectorF(3.5f, 2);
                alternatePositon = new VectorF(3, 2);
                defaultTarget = new VectorF(96, 72);
                alternateTarget = new VectorF(72, 48);
                coneStackPosition = new VectorF(142, 60);

                leftPark = new VectorF(3, 2);
                middlePark = new VectorF(4, 2);
                rightPark = new VectorF(5, 2);
        }

        kai.imu.resetYaw();
        if(isStopRequested) return;
        super.start();

        boolean robotInterrupt = false;

        int conesInStack = 0;

        kai.deadwheels.setTransform(initialTransform, 0); // Set initial location to (1, 0)
        dStar.updateStart(tileToNodeIndex(initialTransform));

        //while(signalOpticalSystem.passes < 5) sleep(10);
        SignalOpticalSystem.SignalOrientation signalOrientation = SignalOpticalSystem.SignalOrientation.MIDDLE;
        //SignalOpticalSystem.SignalOrientation signalOrientation = SignalOpticalSystem.SignalOrientation.RIGHT;//signalOpticalSystem.getSignalOrientation();
        //kai.frontCamera.closeCameraDevice();

        armControl.claw(ClawState.CLOSE);
        armControl.update();
        sleep(1250);
        armControl.setLiftHeight(GoalHeight.HIGH);
        armControl.update();

        moveToTile(Control.tileToNodeIndex(targetTile));

        //turnTo(Math.PI);

        while(getRuntime() < PARK_TIME && !checkInvalid()) {
            armControl.coneStack = conesInStack;
            //armControl.setLiftHeight(Control.GoalHeight.HIGH);
            armControl.update();

            if(!robotInterrupt) {
                moveToTile(defaultPosition);
                /*if(kai.leftDist.getDistance(DistanceUnit.INCH) < 24) {
                    robotInterrupt = true;
                    moveToTile(alternatePositon);
                }*/
            } else {
                moveToTile(alternatePositon);
            }

            /*if(robotInterrupt) {
                armControl.setTarget(alternateTarget);
                while(!armControl.willConeHit(alternateTarget)) {
                    if(checkInvalid()) return;
                    armControl.update();
                }
            } else {
                armControl.setTarget(defaultTarget);
                while(!armControl.willConeHit(defaultTarget)) {
                    if(checkInvalid()) return;
                    armControl.update();
                }
            }*/
            while(kai.liftExtension.isBusy()) armControl.update();
            armControl.setLiftHeight(GoalHeight.MID);
            armControl.update();
            sleep(400);
            armControl.claw(Control.ClawState.OPEN);
            sleep(750);

            if(conesInStack <= 0) break;

            if(getRuntime() > PARK_TIME || checkInvalid()) break;

            armControl.setTarget(coneStackPosition);
            armControl.setLiftHeight(GoalHeight.GROUND);
            armControl.update();
            armControl.update();

            moveToTile(depotPosition);
            while(armControl.getExtensionCurrent() - .5 < armControl.getExtensionTarget()) {
                kai.deadwheels.wheelLoop();
                armControl.update();
            }
            armControl.claw(Control.ClawState.CLOSE);
            sleep(1200);
            conesInStack--;
            break;
        }

        armControl.coneStack = 0;
        armControl.setLiftHeight(GoalHeight.GROUND);
        armControl.setAngleOverride(0.0);
        armControl.update();

        if(side == FieldSide.LEFT) {
            double distance = kai.rightDist.getDistance(DistanceUnit.INCH);
            if(distance <= 28) {
                signalOrientation = SignalOpticalSystem.SignalOrientation.LEFT;
            } else if(distance <= 52) {
                signalOrientation = SignalOpticalSystem.SignalOrientation.MIDDLE;
            } else {
                signalOrientation = SignalOpticalSystem.SignalOrientation.RIGHT;
            }
        } else {
            double distance = kai.leftDist.getDistance(DistanceUnit.INCH);
            if(distance <= 28) {
                signalOrientation = SignalOpticalSystem.SignalOrientation.RIGHT;
            } else if(distance <= 52) {
                signalOrientation = SignalOpticalSystem.SignalOrientation.MIDDLE;
            } else {
                signalOrientation = SignalOpticalSystem.SignalOrientation.LEFT;
            }
        }

        dStar.updateStart(tileToNodeIndex(defaultPosition));

        switch (signalOrientation) {
            case MIDDLE:
                moveToTile(Control.tileToNodeIndex(middlePark));
                break;
            case RIGHT:
                moveToTile(Control.tileToNodeIndex(rightPark));
                break;
            case LEFT:
            default:
                moveToTile(Control.tileToNodeIndex(leftPark));
        }

        requestOpModeStop();
    }
}
