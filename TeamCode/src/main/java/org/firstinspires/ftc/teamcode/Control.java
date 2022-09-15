package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class Control extends OpMode {
    Kai kai;

    double thetaAdjustment = 0;

    public enum DriveMode {
        GLOBAL,
        LOCAL
    }

    public enum IntakeValue {
        ON,
        OFF,
        REVERSE,
        SLOW,
        SLOW_REVERSE
    }

    @Override
    public void init() {
        this.kai = new Kai(hardwareMap);
    }

    public void resetHeading() {
        thetaAdjustment = -kai.getHeading();
    }

    public void drive(double flPower, double frPower, double blPower, double brPower) {
        kai.frontLeft.setPower(flPower);
        kai.frontRight.setPower(frPower);
        kai.backLeft.setPower(blPower);
        kai.backRight.setPower(brPower);
    }

    public void mecanumDrive(double x, double y, double turn, DriveMode mode) {
        // Chose between global or local alignment
        double theta;
        switch(mode) {
            case GLOBAL:
                theta = kai.getHeading() + thetaAdjustment;
            case LOCAL:
            default:
                theta = 0;
        }

        // Rotation of axes to realign from theta
        double xAligned = ((x * Math.cos(theta)) + (y * Math.sin(theta)));
        double yAligned = ((x * (-Math.sin(theta))) + (y * Math.cos(theta)));

        // Calculate the correct motor powers
        double negPowers = (xAligned * 1 + yAligned * 1);
        double posPowers = (xAligned * -1 + yAligned * 1);

        // Apply the motor powers on diagonals
        drive(
                negPowers + turn, posPowers - turn,
                posPowers + turn, negPowers - turn
        );
    }

    public void intake(IntakeValue intakeValue) {
        switch(intakeValue) {
            case ON:
                kai.intake.setPower(1);
            case REVERSE:
                kai.intake.setPower(-1);
            case SLOW:
                kai.intake.setPower(0.5);
            case SLOW_REVERSE:
                kai.intake.setPower(-0.5);
            case OFF:
            default:
                kai.intake.setPower(0);
        }
    }
}
