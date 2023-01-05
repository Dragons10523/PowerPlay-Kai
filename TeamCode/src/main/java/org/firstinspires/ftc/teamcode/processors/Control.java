package org.firstinspires.ftc.teamcode.processors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Kai;

public abstract class Control extends OpMode {
    public Kai kai;
    public ArmControl armControl;

    public boolean isStopRequested = false;

    public enum DriveMode {
        GLOBAL,
        LOCAL
    }

    public enum ClawState {
        OPEN,
        CLOSE
    }

    public enum WristState {
        NORMAL,
        FLIPPED,
        LEFT,
        RIGHT
    }

    public enum GoalHeight {
        HIGH,
        MID,
        LOW,
        GROUND,
        NONE
    }

    public static final GoalHeight[] FIELD_SETUP = {
            GoalHeight.GROUND,GoalHeight.LOW, GoalHeight.GROUND,GoalHeight.LOW, GoalHeight.GROUND,
            GoalHeight.LOW,   GoalHeight.MID, GoalHeight.HIGH,  GoalHeight.MID, GoalHeight.LOW,
            GoalHeight.GROUND,GoalHeight.HIGH,GoalHeight.GROUND,GoalHeight.HIGH,GoalHeight.GROUND,
            GoalHeight.LOW,   GoalHeight.MID, GoalHeight.HIGH,  GoalHeight.MID, GoalHeight.LOW,
            GoalHeight.GROUND,GoalHeight.LOW, GoalHeight.GROUND,GoalHeight.LOW, GoalHeight.GROUND
    };

    public void init() {
        kai = new Kai(hardwareMap);
        armControl = new ArmControl(this);
    }

    public void loop() {
        kai.deadwheels.wheelLoop();
        armControl.update();
    }

    public void stop() {
        if(isStopRequested) return;

        stopAllMovement();
        isStopRequested = true;
    }

    public void mecanumDrive(float x, float y, double turn, DriveMode mode) {
        // Chose between global or local alignment
        double angle;
        switch(mode) {
            case GLOBAL:
                angle = kai.getHeading();
                break;
            case LOCAL:
            default:
                angle = 0;
        }

        x = Math.min(1, Math.max(-1, x));
        y = Math.min(1, Math.max(-1, y));

        VectorF vec = VecUtils.rotateVector(new VectorF(x, y), angle);

        kai.drivetrain.drive(vec.get(0), vec.get(1), turn);
    }

    public static VectorF poleIdxToPos(int poleIndex) {
        return new VectorF(
                24 * (poleIndex % 5f) + 12,
                24 * (5 - (poleIndex / 5f)) + 12
        );
    }

    public static int posToPoleIdx(VectorF pos) {
        int x = (int) pos.get(0);
        int y = (int) pos.get(1);
        return (int) ((x - 12)/24 + 5 * (y - 12)/24);
    }

    public double clawDistance() {
        return kai.clawSensor.getDistance(DistanceUnit.INCH);
    }

    public void orientClaw(WristState wrist) {
        switch(wrist) {
            case LEFT:
                kai.clawTwist.setPosition(0);
                break;
            case RIGHT:
                kai.clawTwist.setPosition(1);
                break;
            case NORMAL:
            default:
                kai.clawTwist.setPosition(0.5);
        }
    }

    public void stopAllMovement() {
        if(kai == null) return;
        kai.liftExtension.setPower(0);
        kai.turntable.setPower(0);
        kai.armLiftA.setPower(0);
        kai.armLiftB.setPower(0);
    }

    public double collapseAngle(double angle) {
        return (((angle % VecUtils.TAU) + VecUtils.TAU) % VecUtils.TAU);
    }

    public double mapAngle(double angle, double offset) {
        return mapAngle(angle, -Math.PI, Math.PI, offset);
    }

    public double mapAngle(double angle, double min, double max, double offset) {
        double dist = max - min;
        return (((angle + offset - min) % dist + dist) % dist) + min;
    }

    public static double squaredHypotenuse(double x, double y) {
        return ((x*x)+(y*y));
    }

    public void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
