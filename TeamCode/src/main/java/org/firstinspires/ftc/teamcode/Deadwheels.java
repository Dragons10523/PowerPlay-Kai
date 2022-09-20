package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Deadwheels {
    private final DcMotor leftYMotor;
    private final DcMotor rightYMotor;
    private final DcMotor XMotor;

    private final double symmetricCircumference;
    private final double asymmetricCircumference;
    private final double inchesPerTick;

    private int leftYPosPrev;
    private int rightYPosPrev;
    private int XPosPrev;

    private long lastUpdateTime;

    double currentX;
    double currentY;
    double currentAngle;

    double xVelocity;
    double yVelocity;
    double angularVelocity;

    // All measurements in inches
    public Deadwheels(DcMotor leftYMotor, DcMotor rightYMotor, DcMotor XMotor, double lateralOffset, double forwardOffset, double inchesPerTick) {
        this.leftYMotor = leftYMotor;
        this.rightYMotor = rightYMotor;
        this.XMotor = XMotor;
        this.symmetricCircumference = Control.TAU * lateralOffset;
        this.asymmetricCircumference = Control.TAU * forwardOffset;
        this.inchesPerTick = inchesPerTick;
        lastUpdateTime = -1;
    }

    public void setTransform(double x, double y, double angle) {
        currentX = x;
        currentY = y;
        currentAngle = angle;
    }

    public void wheelLoop() {
        int leftYPos = leftYMotor.getCurrentPosition();
        int rightYPos = rightYMotor.getCurrentPosition();
        int XPos = XMotor.getCurrentPosition();

        // Invert left so directions are the same
        int leftYDelta = -(leftYPos - leftYPosPrev);
        int rightYDelta = rightYPos - rightYPosPrev;
        int XDelta = XPos - XPosPrev;

        // Positive turning right, negative for left
        double turnDelta = ((((rightYDelta - leftYDelta) / 2d) * inchesPerTick) / symmetricCircumference) * Control.TAU;

        double robotTurnXDelta = (turnDelta / Control.TAU) * asymmetricCircumference;
        double robotXDelta = ((XDelta) * inchesPerTick) - robotTurnXDelta;
        double robotYDelta = (leftYDelta + rightYDelta) * Math.PI;

        double updateAngle = currentAngle + (turnDelta / 2);
        double xGlobalDelta = Math.cos(updateAngle) * robotXDelta + Math.sin(updateAngle) * robotYDelta;
        double yGlobalDelta = -Math.sin(updateAngle) * robotXDelta + Math.cos(updateAngle) * robotYDelta;

        // Calculate velocities
        long currentUpdateTime = System.currentTimeMillis();
        if(lastUpdateTime != -1) {
            double deltaTime = (lastUpdateTime - currentUpdateTime) / 1000f;
            xVelocity = xGlobalDelta / deltaTime;
            yVelocity = yGlobalDelta / deltaTime;
            angularVelocity = turnDelta / deltaTime;
        }
        lastUpdateTime = currentUpdateTime;

        currentX += xGlobalDelta;
        currentY += yGlobalDelta;
        currentAngle += turnDelta;

        leftYPosPrev = leftYPos;
        rightYPosPrev = rightYPos;
        XPosPrev = XPos;
    }
}
