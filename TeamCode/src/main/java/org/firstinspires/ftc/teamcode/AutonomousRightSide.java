package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Auto Right", preselectTeleOp = "Drive")
public class AutonomousRightSide extends AbstractAuto {
    @Override public void loop() {
        if(isStopRequested) return;
        super.loop();

        boolean robotInterrupt = false;

        int conesInStack = 4;

        kai.deadwheels.setTransform(1, 0, 0); // Set initial location to (4, 0)

        while(signalOpticalSystem.passes < 5) sleep(10);
        SignalOpticalSystem.SignalOrientation signalOrientation = signalOpticalSystem.getSignalOrientation();

        while(getRuntime() < 25) {
            if(!robotInterrupt) {
                moveToTile(4, 2);
                if(kai.frontDist.getDistance(DistanceUnit.INCH) < 24) {
                    robotInterrupt = true;
                    moveToTile(3, 2);
                }
            } else {
                moveToTile(3, 2);
            }

            lift(GoalHeight.HIGH);

            if(robotInterrupt) {
                aimClaw(17);
                while(!willConeHit(17)) {
                    sleep(100);
                }
            } else {
                aimClaw(13);
                while (!willConeHit(13)) {
                    sleep(100);
                }
            }
            claw(ClawState.OPEN);
            sleep(250);

            if(conesInStack < 0) break;

            aimAtStack();
            sleep(250);
            liftToStack(conesInStack);
            moveToTile(5, 2);
            aimAtStack();
            claw(ClawState.CLOSE);
            conesInStack--;
        }

        switch (signalOrientation) {
            case MIDDLE:
                moveToTile(4, 1);
                break;
            case RIGHT:
                moveToTile(5, 1);
                break;
            case LEFT:
            default:
                moveToTile(3, 1);
        }

        requestOpModeStop();
    }

    public void aimAtStack() {
        aimClaw(
                Math.atan2(
                        60 - kai.deadwheels.currentY,
                        144 - kai.deadwheels.currentX
                ) - kai.deadwheels.currentAngle
        );
    }
}
