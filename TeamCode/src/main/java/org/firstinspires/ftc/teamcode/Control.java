package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Control extends OpMode {
    Kai kai;

    public static final double TAU = Math.PI * 2;
    public static final double HALF_PI = Math.PI / 2;

    // TODO: Calculate the proper angle to ticks value
    public static final double TURNTABLE_RAD_TO_TICK = 100;

    double thetaAdjustment = 0;

    public enum DriveMode {
        GLOBAL,
        LOCAL
    }

    public enum ClawState {
        OPEN,
        CLOSE
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

    @Override
    public void init() {
        this.kai = new Kai(hardwareMap);
    }

    public void drive(double flPower, double frPower, double blPower, double brPower) {
        kai.frontLeft.setPower(flPower);
        kai.frontRight.setPower(frPower);
        kai.backLeft.setPower(blPower);
        kai.backRight.setPower(brPower);
    }

    public void mecanumDrive(double x, double y, double turn, DriveMode mode) {
        // Chose between global or local alignment
        double angle;
        switch(mode) {
            case GLOBAL:
                angle = kai.getHeading() + thetaAdjustment;
                break;
            case LOCAL:
            default:
                angle = 0;
        }

        // Rotation of axes to realign from the angle
        double xAligned = ((x * Math.cos(angle)) + (y * Math.sin(angle)));
        double yAligned = ((x * (-Math.sin(angle))) + (y * Math.cos(angle)));

        // Calculate the correct motor powers
        double negPowers = (xAligned * 1 + yAligned * 1);
        double posPowers = (xAligned * -1 + yAligned * 1);

        // Apply the motor powers on diagonals
        drive(
                negPowers + turn, posPowers - turn,
                posPowers + turn, negPowers - turn
        );
    }

    public void claw(ClawState clawState) {
        switch(clawState) {
            case CLOSE:
                kai.claw.setPosition(0);
            case OPEN:
            default:
                kai.claw.setPosition(1);
        }
    }

    public void toggleClaw() {
        if(kai.claw.getPosition() != 1) {
            claw(ClawState.CLOSE);
        } else {
            claw(ClawState.OPEN);
        }
    }

    public void lift(GoalHeight liftHeight) {
        switch(liftHeight) {
            case HIGH:
                setLiftHeight(1000);
                break;
            case MID:
                setLiftHeight(667);
                break;
            case LOW:
                setLiftHeight(333);
                break;
            case GROUND:
            case NONE:
            default:
                setLiftHeight(0);
        }
    }

    public void aimClaw(int poleIndex) {
        int poleXIndex = poleIndex % 5;
        int poleYIndex = poleIndex / 5;
        poleYIndex = 4 - poleYIndex;

        double poleX = (poleXIndex + 1) * 24;
        double poleY = (poleYIndex + 1) * 24;

        aimClaw(Math.atan2(
                poleY - kai.deadwheels.currentY,
                poleX - kai.deadwheels.currentX)
                - kai.getHeading()
        );
    }

    public void aimClaw(double angle) {
        angle = collapseAngle(angle);
        kai.turntable.setTargetPosition((int) (angle * TURNTABLE_RAD_TO_TICK));
    }

    public double clawAngle() {
        return kai.turntable.getCurrentPosition() / TURNTABLE_RAD_TO_TICK;
    }

    public void setLiftHeight(int liftHeight) {
        kai.armLiftA.setTargetPosition(liftHeight);
        kai.armLiftB.setTargetPosition(liftHeight);
    }

    public void resetHeading() {
        thetaAdjustment = -kai.getHeading();
    }

    public double collapseAngle(double angle) {
        return (((angle % TAU) + TAU) % TAU);
    }

    public double mapAngle(double angle, double offset) {
        return mapAngle(angle, 0, TAU, offset);
    }

    public double mapAngle(double angle, double min, double max, double offset) {
        return ((((angle + offset) - min) % max + max) % max) + min;
    }

    public double squaredHypotenuse(double x, double y) {
        return ((x*x)+(y*y));
    }
}
