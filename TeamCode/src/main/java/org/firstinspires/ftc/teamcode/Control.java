package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Control extends OpMode {
    Kai kai;

    final double TAU = Math.PI * 2;
    final double HALF_PI = Math.PI / 2;

    double thetaAdjustment = 0;

    public enum DriveMode {
        GLOBAL,
        LOCAL
    }

    public enum ToggleState {
        OPEN,
        CLOSED
    }

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

    public void claw(ToggleState clawState) {
        switch(clawState) {
            case CLOSED:
                kai.claw.setPosition(0);
            case OPEN:
            default:
                kai.claw.setPosition(1);
        }
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
