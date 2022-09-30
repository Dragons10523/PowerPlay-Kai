package org.firstinspires.ftc.teamcode;

public class AbstractAuto extends Control {
    DStar dStar;

    // This is needed to end auto because we are using OpMode now
    @Override
    public void loop() {
        requestOpModeStop();
    }

    @Override
    public void init() {
        super.init();
        this.dStar = new DStar(6, 6, 0, 0);
    }

    // TODO: Do deadwheels and D* math in another thread

    public void moveToTile(int X, int Y) {
        int xInch = X * 24 + 12;
        int yInch = Y * 24 + 12;

        // Align Y to the nearest column
        double yAdjust = 12 - (kai.deadwheels.currentY % 24);
        while(Math.abs(yAdjust) > 4) {
            mecanumDrive(0, yAdjust * .4, 0, DriveMode.GLOBAL);

            yAdjust = 12 - (kai.deadwheels.currentY % 24);
        }

        // Go to the right row
        double xAdjust = xInch - kai.deadwheels.currentX;
        while(Math.abs(xAdjust) > 4) {
            mecanumDrive(xAdjust, yAdjust, 0, DriveMode.GLOBAL) ;

            xAdjust = xInch - kai.deadwheels.currentX;
            yAdjust = 12 - (kai.deadwheels.currentY % 24);
        }

        // Go to the right column
        yAdjust = yInch - kai.deadwheels.currentY;
        while(Math.abs(yAdjust) > 4) {
            mecanumDrive(xAdjust, yAdjust, 0, DriveMode.GLOBAL) ;

            xAdjust = xInch - kai.deadwheels.currentX;
            yAdjust = yInch - kai.deadwheels.currentY;
        }
    }
}
