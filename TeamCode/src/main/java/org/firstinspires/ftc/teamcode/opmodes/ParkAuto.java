package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.processors.AutoControl;
import org.firstinspires.ftc.teamcode.processors.SignalOpticalSystem;

@Autonomous(name = "ParkAuto", preselectTeleOp = "Drive")
public class ParkAuto extends AutoControl {
    public void start() {
        VectorF initialTransform = new VectorF(1.13f, -(2f/24));
        kai.deadwheels.setTransform(initialTransform, 0);
        dStar.updateStart(tileToNodeIndex(initialTransform));

        while(signalOpticalSystem.passes < 5) sleep(10);
        SignalOpticalSystem.SignalOrientation signalOrientation = signalOpticalSystem.getSignalOrientation();
        kai.frontCamera.closeCameraDevice();

        armControl.claw(ClawState.CLOSE);
        sleep(1250);

        switch(signalOrientation) {
            case LEFT:
                moveToTile(new VectorF(0, 1));
                break;
            case MIDDLE:
                moveToTile(new VectorF(1, 1));
                break;
            case RIGHT:
                moveToTile(new VectorF(2, 1));
                break;
        }
    }
}
