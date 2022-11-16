package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.utils.VecUtils;

public class ArmControl extends Thread {
    // TODO: Calculate the proper angle to ticks value
    public static final double TURNTABLE_TICKS_PER_RAD = 100;

    public Control control;
    public boolean shouldRun = false;
    public int coneStack = -1;

    private VectorF target;
    private Control.GoalHeight liftHeight;

    public ArmControl(Control control) {
        this.control = control;
    }

    public void run() {
        try {
            while(!Thread.interrupted()) {
                if(shouldRun) {
                    aimClaw();
                    lift();
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void setTarget(float x, float y) {
        this.target = new VectorF(x, y);
    }

    public void setTarget(VectorF target) {
        this.target = target;
    }

    public void setLiftHeight(Control.GoalHeight liftHeight) {
        this.liftHeight = liftHeight;
    }

    private void aimClaw() {
        if(liftHeight == Control.GoalHeight.GROUND || liftHeight == Control.GoalHeight.NONE) {
            aimClaw(0);
            return;
        }

        VectorF poleVec = this.target.subtracted(new VectorF((float) control.kai.deadwheels.currentX, (float) control.kai.deadwheels.currentY));
        VectorF velVec = new VectorF((float) control.kai.deadwheels.xVelocity, (float) control.kai.deadwheels.yVelocity);

        // Calculate the position to target in field space
        VectorF targetVec = poleVec.subtracted(velVec);

        double currentAngle = control.kai.getHeading();

        // Convert the target position from field space to robot space
        VectorF robotVec = VecUtils.rotateVector(targetVec, currentAngle);

        double targetAngle = VecUtils.getVectorAngle(robotVec);

        // Set the extension distance
        setExtensionDistance(robotVec.length());
        // Aim
        aimClaw(targetAngle);
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
    public void lift() {
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
                if(Math.abs(tableAngle()) > 0.08) {
                    setLiftHeight(300);
                    break;
                }
                setLiftHeight(0);
        }
    }

    public void claw(Control.ClawState clawState) {
        switch(clawState) {
            case CLOSE:
                control.kai.claw.setPosition(0);
            case OPEN:
            default:
                control.kai.claw.setPosition(1);
        }
    }

    public boolean isClawOpen() {
        return control.kai.claw.getPosition() != 0;
    }

    public void toggleClaw() {
        if(!isClawOpen()) {
            claw(Control.ClawState.CLOSE);
        } else {
            claw(Control.ClawState.OPEN);
        }
    }

    public void aimClaw(double angle) {
        angle = control.mapAngle(angle, 0);
        control.kai.turntable.setTargetPosition((int) (angle * TURNTABLE_TICKS_PER_RAD));
    }

    public void setExtensionDistance(double distance) {
        // TODO: Replace 1000 with the proper conversion value
        control.kai.horizontalLift.setTargetPosition((int) (distance * 1000));
    }

    public double getExtensionTarget() {
        // TODO: Don't forget me!
        return control.kai.horizontalLift.getTargetPosition() / 1000.0;
    }

    public double getExtensionCurrent() {
        // TODO: Don't forget me either!
        return control.kai.horizontalLift.getCurrentPosition() / 1000.0;
    }

    public void setLiftHeight(int liftHeight) {
        control.kai.armLiftA.setTargetPosition(liftHeight);
        control.kai.armLiftB.setTargetPosition(liftHeight);
    }

    public double tableAngle() {
        return control.collapseAngle(control.kai.turntable.getCurrentPosition() / TURNTABLE_TICKS_PER_RAD);
    }

    public double tableVel() {
        return control.kai.turntable.getVelocity() / TURNTABLE_TICKS_PER_RAD;
    }
}
