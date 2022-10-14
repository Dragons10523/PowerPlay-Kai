package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Deadwheels {
    private final DcMotor leftYEncoder;
    private final DcMotor rightYEncoder;
    private final DcMotor XEncoder;

    private final double symmetricCircumference;
    private final double asymmetricCircumference;
    private final double inchesPerTick;

    private int leftYPosPrev;
    private int rightYPosPrev;
    private int XPosPrev;

    private long lastUpdateTime;

    public double currentX;
    public double currentY;
    public double currentAngle;

    public double xVelocity;
    public double yVelocity;
    public double angularVelocity;

    // All measurements in inches
    public Deadwheels(DcMotor leftYEncoder, DcMotor rightYEncoder, DcMotor XEncoder, double lateralOffset, double forwardOffset, double inchesPerTick) {
        this.leftYEncoder = leftYEncoder;
        this.rightYEncoder = rightYEncoder;
        this.XEncoder = XEncoder;
        this.symmetricCircumference = Control.TAU * lateralOffset;
        this.asymmetricCircumference = Control.TAU * forwardOffset;
        this.inchesPerTick = inchesPerTick;
        lastUpdateTime = -1;
    }

    public void setTransform(double x, double y, double angle) {
        currentX = (x * 24) + 12;
        currentY = (y * 24) + 12;
        currentAngle = angle;
        lastUpdateTime = -1;
    }

    public void wheelLoop() {
        int leftYPos = leftYEncoder.getCurrentPosition();
        int rightYPos = rightYEncoder.getCurrentPosition();
        int XPos = XEncoder.getCurrentPosition();

        // Invert left so directions are the same
        int leftYDelta = leftYPos - leftYPosPrev;
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
