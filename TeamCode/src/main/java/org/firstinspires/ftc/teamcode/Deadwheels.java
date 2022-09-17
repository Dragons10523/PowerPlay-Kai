package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Deadwheels {
    private DcMotor leftYMotor;
    private DcMotor rightYMotor;
    private DcMotor XMotor;

    double symmetricCircumference;
    double asymmetricCircumference;
    double inchesPerTick;

    int leftYPosPrev;
    int rightYPosPrev;
    int XPosPrev;

    double lastX;
    double lastY;
    double lastAngle;

    // All measurements in inches
    public Deadwheels(DcMotor leftYMotor, DcMotor rightYMotor, DcMotor XMotor, double lateralOffset, double forwardOffset, double inchesPerTick) {
        this.leftYMotor = leftYMotor;
        this.rightYMotor = rightYMotor;
        this.XMotor = XMotor;
        this.symmetricCircumference = Control.TAU * lateralOffset;
        this.asymmetricCircumference = Control.TAU * forwardOffset;
        this.inchesPerTick = inchesPerTick;
    }

    public void setTransform(double x, double y, double angle) {
        lastX = x;
        lastY = y;
        lastAngle = angle;
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

        double updateAngle = lastAngle + (turnDelta / 2);
        double xGlobalDelta = Math.cos(updateAngle) * robotXDelta + Math.sin(updateAngle) * robotYDelta;
        double yGlobalDelta = -Math.sin(updateAngle) * robotXDelta + Math.cos(updateAngle) * robotYDelta;

        lastX += xGlobalDelta;
        lastY += yGlobalDelta;
        lastAngle += turnDelta;

        leftYPosPrev = leftYPos;
        rightYPosPrev = rightYPos;
        XPosPrev = XPos;
    }
}
