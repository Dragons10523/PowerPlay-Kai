package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.processors.AutoControl;
import org.firstinspires.ftc.teamcode.processors.Control;
import org.firstinspires.ftc.teamcode.processors.SignalOpticalSystem;

public abstract class Auto extends AutoControl {
    public void auto(FieldSide side) {
        VectorF initialTransform;
        VectorF defaultPosition;
        VectorF alternatePositon;
        VectorF defaultTarget;
        VectorF alternateTarget;
        VectorF coneStackPosition;

        switch(side) {
            case LEFT:
                initialTransform = new VectorF(1, 0);
                defaultPosition = new VectorF(1, 2);
                alternatePositon = new VectorF(2, 2);
                defaultTarget = new VectorF(1, 2);
                alternateTarget = new VectorF(2, 1);
                coneStackPosition = new VectorF(2, 60);
                break;
            case RIGHT:
            default:
                initialTransform = new VectorF(4, 0);
                defaultPosition = new VectorF(4, 2);
                alternatePositon = new VectorF(3, 2);
                defaultTarget = new VectorF(4, 2);
                alternateTarget = new VectorF(2, 1);
                coneStackPosition = new VectorF(142, 60);
        }

        if(isStopRequested) return;
        super.start();

        boolean robotInterrupt = false;

        int conesInStack = 4;

        kai.deadwheels.setTransform(initialTransform, 0); // Set initial location to (1, 0)

        while(signalOpticalSystem.passes < 5) sleep(10);
        SignalOpticalSystem.SignalOrientation signalOrientation = signalOpticalSystem.getSignalOrientation();
        kai.frontCamera.closeCameraDevice();

        while(getRuntime() < 25) {
            if(!robotInterrupt) {
                moveToTile(Control.posToPoleIdx(defaultPosition));
                if(kai.frontDist.getDistance(DistanceUnit.INCH) < 24) {
                    robotInterrupt = true;
                    moveToTile(Control.posToPoleIdx(alternatePositon));
                }
            } else {
                moveToTile(Control.posToPoleIdx(alternatePositon));
            }
            armControl.setLiftHeight(Control.GoalHeight.HIGH);

            if(robotInterrupt) {
                armControl.setTarget(alternateTarget);
                while(!armControl.willConeHit(alternateTarget)) {
                    sleep(100);
                }
            } else {
                armControl.setTarget(defaultTarget);
                while(!armControl.willConeHit(defaultTarget)) {
                    sleep(100);
                }
            }
            armControl.claw(Control.ClawState.OPEN);
            sleep(250);

            if(conesInStack < 0) break;

            armControl.setTarget(coneStackPosition);
            sleep(250);
            liftToStack(conesInStack);
            moveToTile(0, 2);
            armControl.setTarget(coneStackPosition);
            armControl.claw(Control.ClawState.CLOSE);
            conesInStack--;
        }

        switch (signalOrientation) {
            case MIDDLE:
                moveToTile(1, 1);
                break;
            case RIGHT:
                moveToTile(2, 1);
                break;
            case LEFT:
            default:
                moveToTile(0, 1);
        }

        requestOpModeStop();
    }
}
