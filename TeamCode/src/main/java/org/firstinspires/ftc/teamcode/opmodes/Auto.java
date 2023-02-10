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

        while(signalOpticalSystem.passes < 5) sleep(10);
        SignalOpticalSystem.SignalOrientation signalOrientation = signalOpticalSystem.getSignalOrientation();
        kai.frontCamera.closeCameraDevice();

        armControl.claw(ClawState.CLOSE);
        armControl.sleep(1250);

        moveToTile(Control.tileToNodeIndex(targetTile));

        while(getRuntime() < PARK_TIME && !checkInvalid()) {
            armControl.coneStack = conesInStack;

            /*if(!robotInterrupt) {
                moveToTile(defaultPosition);
                /*if(kai.leftDist.getDistance(DistanceUnit.INCH) < 24) {
                    robotInterrupt = true;
                    moveToTile(alternatePositon);
                }*/
            /*} else {
                moveToTile(alternatePositon);
            }*/

            armControl.setLiftHeight(Control.GoalHeight.HIGH);
            armControl.setAngleOverride(null);
            armControl.setTarget(defaultTarget);
            armControl.sleep(100);
            turnTo(Math.atan2(
                    defaultTarget.get(0)-kai.deadwheels.currentX,
                    defaultTarget.get(1)-kai.deadwheels.currentY
            ));

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
            //while(kai.liftExtension.isBusy()) armControl.update();
            armControl.setLiftHeight(GoalHeight.MID);
            armControl.sleep(800);
            armControl.claw(Control.ClawState.OPEN);
            armControl.sleep(750);
            armControl.setTarget(null);
            armControl.setExtensionDistance(0);
            armControl.sleep(150);

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
            armControl.sleep(1200);
            conesInStack--;
            break;
        }

        armControl.coneStack = 0;
        armControl.setLiftHeight(GoalHeight.GROUND);
        armControl.setAngleOverride(0.0);
        armControl.update();

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
