package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class Control extends OpMode {
    Kai kai;
    DStar dStar;

    public static final double TAU = Math.PI * 2;
    public static final double HALF_PI = Math.PI / 2;

    // TODO: Calculate the proper angle to ticks value
    public static final double TURNTABLE_TICKS_PER_RAD = 100;

    public enum DriveMode {
        GLOBAL,
        LOCAL
    }

    public enum ClawState {
        OPEN,
        CLOSE
    }

    public enum FlipState {
        FORWARDS,
        BACKWARDS
    }

    public enum WristState {
        NORMAL,
        FLIPPED
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
        this.dStar = new DStar(6, 6, 0, 0);
    }

    @Override
    public void loop() {
        kai.deadwheels.wheelLoop();
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
                angle = kai.getHeading();
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
        if(!clawOpen()) {
            claw(ClawState.CLOSE);
        } else {
            claw(ClawState.OPEN);
        }
    }

    public boolean clawOpen() {
        return kai.claw.getPosition() != 0;
    }

    public double clawDistance() {
        return kai.clawSensor.getDistance(DistanceUnit.INCH);
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

        // Calculate the pole position
        double poleX = (poleXIndex + 1) * 24;
        double poleY = (poleYIndex + 1) * 24;

        // RC means robot centered
        double poleXRC = poleX - kai.deadwheels.currentX;
        double poleYRC = poleY - kai.deadwheels.currentY;

        double velX = kai.deadwheels.xVelocity;
        double velY = kai.deadwheels.yVelocity;

        // Calculate the position to target in field space
        double targetX = poleXRC - velX;
        double targetY = poleYRC - velY;

        double currentAngle = kai.getHeading();

        // Convert the target position from field space to robot space
        double robotX = Math.cos(currentAngle) * targetX + Math.sin(currentAngle) * targetY;
        double robotY = -Math.sin(currentAngle) * targetX + Math.cos(currentAngle) * targetY;

        double targetAngle = Math.atan2(robotX, robotY);

        // Claw Flipping
        boolean flippedValues = false;
        int extensionMult = 1;
        int angleOffset = 0;
        if(Math.abs(mapAngle(targetAngle - tableAngle(), 0)) > 95) {
            flippedValues = true;
        }
        if(getExtensionTarget() < 0) {
            flippedValues = !flippedValues;
        }
        if(flippedValues) {
            extensionMult = -1;
            angleOffset = 180;
            flipClaw(FlipState.BACKWARDS);
        } else {
            flipClaw(FlipState.FORWARDS);
        }

        // Set the extension distance
        setExtensionDistance(extensionMult * Math.hypot(robotX, robotY));
        // Aim
        aimClaw(angleOffset + targetAngle);
    }

    public void aimClaw(double angle) {
        angle = mapAngle(angle, 0);
        kai.turntable.setTargetPosition((int) (angle * TURNTABLE_TICKS_PER_RAD));
    }

    public double tableAngle() {
        double angleMultiplier = 1;
        if(getExtensionTarget() < 0) {
            angleMultiplier *= -1;
        }
        return collapseAngle(angleMultiplier * kai.turntable.getCurrentPosition() / TURNTABLE_TICKS_PER_RAD);
    }

    public double tableVel() {
        return kai.turntable.getVelocity() / TURNTABLE_TICKS_PER_RAD;
    }

    public boolean willConeHit(int poleIndex) {
        double xVel = kai.deadwheels.xVelocity;
        double yVel = kai.deadwheels.yVelocity;
        double xPos = kai.deadwheels.currentX;
        double yPos = kai.deadwheels.currentY;

        double clawAngle = collapseAngle(-kai.getHeading() + tableAngle());
        double clawRotVel = kai.deadwheels.angularVelocity + tableVel();

        double clawExtension = getExtensionCurrent();
        double clawXVel = Math.cos(clawAngle) * clawRotVel * TAU * clawExtension;
        double clawYVel = -Math.sin(clawAngle) * clawRotVel * TAU * clawExtension;
        xVel += clawXVel;
        yVel += clawYVel;

        double clawXPos = Math.cos(clawAngle) * TAU * clawExtension;
        double clawYPos = -Math.sin(clawAngle) * TAU * clawExtension;
        xPos += clawXPos;
        yPos += clawYPos;

        int poleXIndex = poleIndex % 5;
        int poleYIndex = poleIndex / 5;
        poleYIndex = 4 - poleYIndex;

        double poleX = (poleXIndex + 1) * 24;
        double poleY = (poleYIndex + 1) * 24;

        double hitX = xPos + xVel * 0.1 - poleX;
        double hitY = yPos + yVel * 0.1 - poleY;

        return squaredHypotenuse(hitX, hitY) <= 0.6;
    }

    public void setLiftHeight(int liftHeight) {
        kai.armLiftA.setTargetPosition(liftHeight);
        kai.armLiftB.setTargetPosition(liftHeight);
    }

    public void setExtensionDistance(double distance) {
        // TODO: Replace 1000 with the proper conversion value
        kai.horizontalLift.setTargetPosition((int) (distance * 1000));
    }

    public double getExtensionTarget() {
        // TODO: Don't forget me!
        return kai.horizontalLift.getTargetPosition() / 1000.0;
    }

    public double getExtensionCurrent() {
        // TODO: Don't forget me either!
        return kai.horizontalLift.getCurrentPosition() / 1000.0;
    }

    public void flipClaw(FlipState flip) {
        switch(flip) {
            case BACKWARDS:
                kai.clawFlup.setPosition(1);
                break;
            case FORWARDS:
            default:
                kai.clawFlup.setPosition(0);
        }
    }

    public void orientClaw(WristState wrist) {
        int backwardsOffset = 0;
        if(kai.clawFlup.getPosition() != 0) {
            backwardsOffset = 1;
        }

        switch(wrist) {
            case FLIPPED:
                kai.clawTwist.setPosition(1 - backwardsOffset);
                break;
            case NORMAL:
            default:
                kai.clawTwist.setPosition(backwardsOffset);
        }
    }

    public double collapseAngle(double angle) {
        return (((angle % TAU) + TAU) % TAU);
    }

    public double mapAngle(double angle, double offset) {
        return mapAngle(angle, -Math.PI, Math.PI, offset);
    }

    public double mapAngle(double angle, double min, double max, double offset) {
        return ((((angle + offset) - min) % max + max) % max) + min;
    }

    public double squaredHypotenuse(double x, double y) {
        return ((x*x)+(y*y));
    }
}
