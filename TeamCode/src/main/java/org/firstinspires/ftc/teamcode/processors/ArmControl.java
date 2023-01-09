package org.firstinspires.ftc.teamcode.processors;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

// This class is meant to handle "blocking" to prevent any possibly dangerous movements from happening
public class ArmControl {
    // TODO: Calculate the proper angle to ticks value
    public static final double TURNTABLE_TICKS_PER_RAD = 3024/VecUtils.TAU;
    // TODO: Replace 1000 with the proper conversion value
    public static final double EXTENSION_TICKS_PER_INCH = 88.9;

    public static final int GROUND_GOAL_HEIGHT = 0;
    public static final int LOW_GOAL_HEIGHT = 3100;
    public static final int MID_GOAL_HEIGHT = 4500;
    public static final int HIGH_GOAL_HEIGHT = 6300;

    public Control control;
    public int coneStack = 0;

    private VectorF target = null;
    private Control.GoalHeight liftHeight = Control.GoalHeight.NONE;
    private Double angleOverride = null;
    private boolean liftHeightChanged = false;
    private int extensionDistance = 0;
    private boolean clawLastOpened = true;

    public ArmControl(Control control) {
        this.control = control;
    }

    public void update() {
        aimClaw();
        lift();
        moveExtensionWhenSafe();
    }

    public void setTarget(float x, float y) {
        this.target = new VectorF(x, y);
    }

    public void setTarget(VectorF target) {
        this.target = target;
    }

    public void setAngleOverride(Double angle) {
        angleOverride = angle;
    }

    public void setLiftHeight(Control.GoalHeight liftHeight) {
        this.liftHeight = liftHeight;
        liftHeightChanged = true;
    }

    private void aimClaw() {
        // Blocked by the lift target
        boolean limitAngle = liftHeight == Control.GoalHeight.GROUND || liftHeight == Control.GoalHeight.NONE || getLiftCurrentLiftHeight() < 1800;

        double targetAngle;

        System.out.println("Aiming");
        if(angleOverride == null) {
            System.out.println("No Override");
            if(target == null) {
                return;
            }
            System.out.println("Target Acquired");

            VectorF poleVec = this.target.subtracted(new VectorF((float) control.kai.deadwheels.currentX, (float) control.kai.deadwheels.currentY));
            VectorF velVec = new VectorF((float) control.kai.deadwheels.xVelocity, (float) control.kai.deadwheels.yVelocity);

            // Calculate the position to target in field space
            VectorF targetVec = poleVec.subtracted(velVec);

            System.out.println("Pole Vector: " + poleVec);
            System.out.println("Velocity Vector: " + velVec);
            System.out.println("Target Vector: " + targetVec);

            double currentAngle = control.kai.getHeading();

            // Convert the target position from field space to robot space
            VectorF robotVec = VecUtils.rotateVector(targetVec, currentAngle);

            targetAngle = VecUtils.getVectorAngle(robotVec);

            extensionDistance = (int) ((Math.hypot(robotVec.get(0), robotVec.get(1)) - 9.75) * EXTENSION_TICKS_PER_INCH);
        } else {
            targetAngle = angleOverride;
        }

        targetAngle = (limitAngle ? Math.min(-.3, Math.max(.3, targetAngle)) : targetAngle);

        // Set the extension distance
        // Aim
        //aimClaw(targetAngle);
    }

    public boolean willConeHit(VectorF target) {
        return willConeHit(Control.posToPoleIdx(target));
    }

    public boolean willConeHit(int poleIndex) {
        double xVel = control.kai.deadwheels.xVelocity;
        double yVel = control.kai.deadwheels.yVelocity;
        double xPos = control.kai.deadwheels.currentX;
        double yPos = control.kai.deadwheels.currentY;

        double clawAngle = control.collapseAngle(-control.kai.getHeading() + tableAngle());
        double clawRotVel = control.kai.deadwheels.angularVelocity + tableVel();

        double clawExtension = getExtensionCurrent();
        double clawXVel = Math.cos(clawAngle) * clawRotVel * VecUtils.TAU * clawExtension;
        double clawYVel = -Math.sin(clawAngle) * clawRotVel * VecUtils.TAU * clawExtension;
        xVel += clawXVel;
        yVel += clawYVel;

        double clawXPos = Math.cos(clawAngle) * VecUtils.TAU * clawExtension;
        double clawYPos = -Math.sin(clawAngle) * VecUtils.TAU * clawExtension;
        xPos += clawXPos;
        yPos += clawYPos;

        int poleXIndex = poleIndex % 5;
        int poleYIndex = poleIndex / 5;
        poleYIndex = 4 - poleYIndex;

        double poleX = (poleXIndex + 1) * 24;
        double poleY = (poleYIndex + 1) * 24;

        double hitX = xPos + xVel * 0.1 - poleX;
        double hitY = yPos + yVel * 0.1 - poleY;

        return Control.squaredHypotenuse(hitX, hitY) <= 0.6;
    }

    // TODO: calibrate lift heights
    private void lift() {
        // Blocked by the extension's current position
        if(getExtensionCurrent() >= 2) {
            return;
        }

        switch(liftHeight) {
            case HIGH:
                setLiftHeight(HIGH_GOAL_HEIGHT);
                break;
            case MID:
                setLiftHeight(MID_GOAL_HEIGHT);
                break;
            case LOW:
                setLiftHeight(LOW_GOAL_HEIGHT);
                break;
            case GROUND:
            case NONE:
            default:
                if(Math.abs(tableAngle()) > 0.4) {
                    setLiftHeight(LOW_GOAL_HEIGHT);
                    break;
                }

                coneStack = Math.min(5, Math.max(1, coneStack));
                setLiftHeight((250 * coneStack) - 250);
        }
    }

    public void claw(Control.ClawState clawState) {
        switch(clawState) {
            case CLOSE:
                control.kai.claw.setPosition(0);
                clawLastOpened = false;
                break;
            case OPEN:
            default:
                control.kai.claw.setPosition(1);
                clawLastOpened = true;
        }
    }

    public boolean isClawOpen() {
        return clawLastOpened;
    }

    public void toggleClaw() {
        if(isClawOpen()) {
            claw(Control.ClawState.CLOSE);
        } else {
            claw(Control.ClawState.OPEN);
        }
    }

    private void aimClaw(double angle) {
        angle = control.mapAngle(angle, 0);
        control.kai.turntable.setTargetPosition((int) (angle * TURNTABLE_TICKS_PER_RAD));
    }

    private void moveExtensionWhenSafe() {
        // Lift blocks all
        extensionDistance = Math.min(1333, Math.max(0, extensionDistance));
        System.out.println("Checking Extension");
        if(getLiftCurrentLiftHeight() > getGoalHeight(liftHeight) - 800) {
            System.out.println("Extendo Patronum at " + extensionDistance);
            control.kai.liftExtension.setTargetPosition(extensionDistance);
        } else if(liftHeightChanged) {
            System.out.println("No Extendo");
            control.kai.liftExtension.setTargetPosition(0);
        }
    }

    public void setExtensionDistance(double distance) {
        extensionDistance = (int)(distance * EXTENSION_TICKS_PER_INCH);
    }

    public double getExtensionTarget() {
        return extensionDistance / EXTENSION_TICKS_PER_INCH;
    }

    public double getExtensionCurrent() {
        return control.kai.liftExtension.getCurrentPosition() / EXTENSION_TICKS_PER_INCH;
    }

    private void setLiftHeight(int liftHeight) {
        control.kai.armLiftA.setTargetPosition(liftHeight);
        control.kai.armLiftB.setTargetPosition(liftHeight);
        liftHeightChanged = false;
    }

    public int getLiftCurrentLiftHeight() {
        return control.kai.armLiftA.getCurrentPosition();
    }

    public double tableAngle() {
        return control.mapAngle(control.kai.turntable.getCurrentPosition() / TURNTABLE_TICKS_PER_RAD, -Math.PI, Math.PI, 0);
    }

    public double tableVel() {
        return control.kai.turntable.getVelocity() / TURNTABLE_TICKS_PER_RAD;
    }

    public int getGoalHeight(Control.GoalHeight goalHeight) {
        switch (goalHeight) {
            case LOW:
                return LOW_GOAL_HEIGHT;
            case MID:
                return MID_GOAL_HEIGHT;
            case HIGH:
                return HIGH_GOAL_HEIGHT;
            case NONE:
            case GROUND:
            default:
                return GROUND_GOAL_HEIGHT;
        }
    }
}
