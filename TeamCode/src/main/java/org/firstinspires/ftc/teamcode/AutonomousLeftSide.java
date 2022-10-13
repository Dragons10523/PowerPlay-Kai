package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Left", preselectTeleOp = "Drive")
public class AutonomousLeftSide extends AbstractAuto {
    @Override public void loop() {
        if(isStopRequested) return;
        super.loop();

        int conesInStack = 5;

        kai.deadwheels.setTransform(1, 0, 0); // Set initial location to (4, 0)

        while(!signalOpticalSystem.isReady()) sleep(10);
        SignalOpticalSystem.SignalOrientation signalOrientation = signalOpticalSystem.getSignalOrientation();

        while(getRuntime() < 25) {
            moveToTile(1, 2);
            lift(GoalHeight.HIGH);
            aimClaw(11);
            while(!willConeHit(11)) {
                sleep(100);
            }
            claw(ClawState.OPEN);
            sleep(250);

            if(conesInStack <= 0) break;

            aimAtStack();
            sleep(250);
            liftToStack(conesInStack);
            moveToTile(0, 2);
            aimAtStack();
            claw(ClawState.CLOSE);
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

    public void aimAtStack() {
        aimClaw(
                Math.atan2(
                        60 - kai.deadwheels.currentY,
                        0 - kai.deadwheels.currentX
                ) - kai.deadwheels.currentAngle
        );
    }
}
