package org.firstinspires.ftc.teamcode.processors;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

public abstract class AutoControl extends Control {
    DStar dStar;

    protected SignalOpticalSystem signalOpticalSystem;
    private DistanceSensor[] robotSensors;
    private double[][] sensorOffsets;

    public enum FieldSide {
        LEFT,
        RIGHT
    }

    @Override
    public void init() {
        telemetry.addLine("Robot is Starting...");
        telemetry.update();

        signalOpticalSystem = new SignalOpticalSystem();

        telemetry.addLine("Opening camera...");
        telemetry.update();
        kai.frontCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                kai.frontCamera.startStreaming(SignalOpticalSystem.CAMERA_WIDTH, SignalOpticalSystem.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @SuppressLint("DefaultLocale")
            @Override
            public void onError(int errorCode) {
                telemetry.addLine(String.format("Robot startup failed with error code %d", errorCode));
                telemetry.update();
            }
        });

        telemetry.addLine("Setting up systems...");
        telemetry.update();
        kai.frontCamera.setPipeline(signalOpticalSystem);
        super.init();
        this.dStar = new DStar(6, 6, 0, 0);

        robotSensors = new DistanceSensor[]{kai.frontDist, kai.rightDist, kai.leftDist, kai.backDist};
        // X, Y, Angle
        sensorOffsets = new double[][]{
                {0, 0, -VecUtils.HALF_PI},
                {0, 0, 0},
                {0, 0, Math.PI},
                {0, 0, VecUtils.HALF_PI}
        };

        while(!signalOpticalSystem.isReady()) sleep(100);
        telemetry.addLine("Robot is Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        requestOpModeStop();
    }

    public void liftToStack(int stackHeight) {
        // TODO: Calibrate real values
        armControl.coneStack = 1000 + (10 * stackHeight);
    }

    public void moveToTile(int nodeIndex) {
        if(isStopRequested) return;

        this.dStar.updateEnd(nodeIndex);
        checkPathing();

        List<Integer> compressedPath = dStar.getCompressedPath();

        for (Integer cornerNode : compressedPath) {
            int nodeX = cornerNode % 6;
            int nodeY = (int) Math.floor(cornerNode / 6f);

            if(moveToTile(nodeX, nodeY)) {
                moveToTile(nodeIndex);
                return;
            }
        }
    }

    // Returns true on robot detected
    public boolean moveToTile(int X, int Y) {
        if(isStopRequested) return false; // Returning false so we don't get Stack Overflows from recursion

        int xInch = X * 24 + 12;
        int yInch = Y * 24 + 12;

        // Align Y to the nearest column
        if(checkPathing()) return true;
        double yAdjust = 12 - (kai.deadwheels.currentY % 24);
        while(Math.abs(yAdjust) > 4) {
            if(isStopRequested) return false;

            mecanumDrive(0, (float) (yAdjust * .4), 0, DriveMode.GLOBAL);

            if(checkPathing()) return true;
            yAdjust = 12 - (kai.deadwheels.currentY % 24);
        }

        // Go to the right row
        double xAdjust = xInch - kai.deadwheels.currentX;
        while(Math.abs(xAdjust) > 4) {
            if(isStopRequested) return false;

            mecanumDrive((float) xAdjust, (float) yAdjust, 0, DriveMode.GLOBAL) ;

            if(checkPathing()) return true;
            xAdjust = xInch - kai.deadwheels.currentX;
            yAdjust = 12 - (kai.deadwheels.currentY % 24);
        }

        // Go to the right column
        yAdjust = yInch - kai.deadwheels.currentY;
        while(Math.abs(yAdjust) > 4) {
            if(isStopRequested) return false;

            mecanumDrive((float) xAdjust, (float) yAdjust, 0, DriveMode.GLOBAL) ;

            if(checkPathing()) return true;
            xAdjust = xInch - kai.deadwheels.currentX;
            yAdjust = yInch - kai.deadwheels.currentY;
        }

        return false;
    }

    public VectorF getSensorIntercept(DistanceSensor sensor, double[] sensorOffset) {
        double distance = sensor.getDistance(DistanceUnit.INCH);
        double robotAngle = kai.getHeading();

        // Calculate the origin intercept
        VectorF intercept = new VectorF(
                (float)(Math.cos(robotAngle + sensorOffset[3]) * distance + kai.deadwheels.currentX),
                (float)(Math.sin(robotAngle + sensorOffset[3]) * distance + kai.deadwheels.currentY));

        // Offset the origin intercept by the sensor position
        intercept.add(VecUtils.rotateVector(new VectorF((float)sensorOffset[0], (float)sensorOffset[1]), robotAngle));

        return intercept;
    }

    // Returns true on robot detected
    private boolean checkPathing() {
        kai.deadwheels.wheelLoop();

        for(int sensorIndex = 0; sensorIndex < 4; sensorIndex++) {
            VectorF intercept = getSensorIntercept(robotSensors[sensorIndex], sensorOffsets[sensorIndex]);

            double xTile = intercept.get(0) / 24;
            double yTile = 6 - (intercept.get(1) / 24);

            if(robotSensors[sensorIndex].getDistance(DistanceUnit.INCH) < 60)
                if(tryBlockPath(xTile, yTile)) return true;

            tryClearPath(xTile, yTile);
        }
        return false;
    }

    private boolean tryBlockPath(double xTile, double yTile) {
        double xMargin = 1 - ((xTile - 0.5) % 1);
        double yMargin = 1 - ((yTile - 0.5) % 1);

        // Don't trust values where the intercept is within 1/8 of a tile edge
        if(xMargin >= 0.125 && yMargin >= 0.125) {
            int nodeIndex = ((int) Math.floor(yTile)) * 6 + ((int) Math.floor(xTile));

            int robotStartIndex = (int)(kai.deadwheels.currentY / 24) * 6 + (int)(kai.deadwheels.currentX / 24);
            dStar.updateStart(robotStartIndex);
            dStar.markBlocked(nodeIndex);
            return true;
        }

        return false;
    }

    private void tryClearPath(double xTile, double yTile) {
        for(Integer nodeIndex : dStar.obstacles) {
            double obstacleX = nodeIndex % 6;
            double obstacleY = Math.floor(nodeIndex / 6f);

            double robotXTile = kai.deadwheels.currentX / 24;
            double robotYTile = 6 - (kai.deadwheels.currentY / 24);

            // Get minimum distance between sensor line and tiles marked as obstacles
            double xOffset = xTile - robotXTile;
            double yOffset = yTile - robotYTile;

            // Calculate the beginning and end points of a the line between the robot and the obstacle location
            double endObstacleXOffset = obstacleX - xTile;
            double endObstacleYOffset = obstacleY - yTile;

            double startObstacleXOffset = obstacleX - robotXTile;
            double startObstacleYOffset = obstacleY - robotYTile;

            // Begin calculating the distance from the detected point to the previously defined line
            double distanceB = (xOffset * endObstacleXOffset + yOffset * endObstacleYOffset);
            double distanceA = (xOffset * startObstacleXOffset + yOffset * startObstacleYOffset);

            double finalDistance;

            if(distanceB < 0) {
                finalDistance = Math.hypot(obstacleY - yTile, obstacleX - xTile);
            } else if(distanceA < 0) {
                finalDistance = Math.hypot(obstacleY - robotYTile, obstacleX - robotXTile);
            } else {
                double mod = Math.hypot(xOffset, yOffset);
                finalDistance = Math.abs(xOffset * startObstacleYOffset - yOffset * startObstacleXOffset) / mod;
            }

            // If we are within an 20 inch diameter circle of the center of the tile, that tile is marked free
            if(finalDistance < 10) {
                dStar.markOpen(nodeIndex);
            }
        }
    }
}
