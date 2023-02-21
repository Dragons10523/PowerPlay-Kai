package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.processors.AutoControl;
import org.firstinspires.ftc.teamcode.processors.SignalOpticalSystem;

@Autonomous(name = "ParkAuto", preselectTeleOp = "Drive")
public class ParkAuto extends AutoControl {
    public void start() {
        kai.deadwheels.wheelLoop();
        VectorF initialTransform = new VectorF(1.13f, -(3.5f/24));
        kai.deadwheels.setTransform(initialTransform, 0);

        SignalOpticalSystem.SignalOrientation signalOrientation = signalOpticalSystem.getSignalOrientation();
        kai.frontCamera.closeCameraDevice();

        armControl.claw(ClawState.CLOSE);
        sleep(1250);
        armControl.setLiftHeight(GoalHeight.LOW);
        armControl.sleep(100);

        moveToTile(new VectorF(1, 0));

        switch(signalOrientation) {
            case LEFT:
                moveToTile(new VectorF(0, 0-(1.5f/24)));
                moveToTile(new VectorF(0, 1));
                break;
            case MIDDLE:
                moveToTile(new VectorF(1, 1));
                break;
            case RIGHT:
                moveToTile(new VectorF(2, 0-(1.5f/24)));
                moveToTile(new VectorF(2, 1));
                break;
        }

        armControl.setLiftHeight(GoalHeight.GROUND);
        armControl.sleep(500);
        armControl.claw(ClawState.OPEN);
        armControl.sleep(500);
    }
}
