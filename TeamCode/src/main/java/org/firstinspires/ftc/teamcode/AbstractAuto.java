package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

public abstract class AbstractAuto extends Control {
    DStar dStar;

    protected SignalOpticalSystem signalOpticalSystem;
    private DistanceSensor[] robotSensors;
    private double[][] sensorOffsets;

    @Override
    public void init() {
        super.init();
        this.dStar = new DStar(6, 6, 0, 0);

        robotSensors = new DistanceSensor[]{kai.frontDist, kai.rightDist, kai.leftDist, kai.backDist};
        // X, Y, Angle
        sensorOffsets = new double[][]{
                {0, 0, -Control.HALF_PI},
                {0, 0, 0},
                {0, 0, Math.PI},
                {0, 0, Control.HALF_PI}
        };

        signalOpticalSystem = new SignalOpticalSystem();
    }

    public void liftToStack(int stackHeight) {
        // TODO: Calibrate real values
        setLiftHeight(1000 + (10 * stackHeight));
    }

    public void moveToTile(int nodeIndex) {
        if(isStopRequested) return;

        this.dStar.updateEnd(nodeIndex);
        checkPathing();

        List<Integer> compressedPath = new ArrayList<>();

        {
            int tileX;
            int tileY;
            int prevTileX = -1;
            int prevTileY = -1;
            int prevDirection = 0; // 1 for X, 2 for Y

            List<Integer> fullPath = this.dStar.getFullPath();

            // Add the first node
            compressedPath.add(fullPath.get(0));

            for (int i = 0; i < fullPath.size(); i++) {
                int nodeCheck = fullPath.get(i);

                tileX = nodeCheck % 6;
                tileY = (int) Math.floor(nodeCheck / 6f);

                // Check if the direction of the path changed, and if so add the corner node
                if (tileX == prevTileX) {
                    if (prevDirection == 2) {
                        compressedPath.add(fullPath.get(i - 1));
                    }
                    prevDirection = 1;
                } else if (tileY == prevTileY) {
                    if (prevDirection == 1) {
                        compressedPath.add(fullPath.get(i - 1));
                    }
                    prevDirection = 2;
                }

                prevTileX = tileX;
                prevTileY = tileY;
            }

            // Add the last node
            compressedPath.add(fullPath.get(fullPath.size() - 1));
        }

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

            mecanumDrive(0, yAdjust * .4, 0, DriveMode.GLOBAL);

            if(checkPathing()) return true;
            yAdjust = 12 - (kai.deadwheels.currentY % 24);
        }

        // Go to the right row
        double xAdjust = xInch - kai.deadwheels.currentX;
        while(Math.abs(xAdjust) > 4) {
            if(isStopRequested) return false;

            mecanumDrive(xAdjust, yAdjust, 0, DriveMode.GLOBAL) ;

            if(checkPathing()) return true;
            xAdjust = xInch - kai.deadwheels.currentX;
            yAdjust = 12 - (kai.deadwheels.currentY % 24);
        }

        // Go to the right column
        yAdjust = yInch - kai.deadwheels.currentY;
        while(Math.abs(yAdjust) > 4) {
            if(isStopRequested) return false;

            mecanumDrive(xAdjust, yAdjust, 0, DriveMode.GLOBAL) ;

            if(checkPathing()) return true;
            xAdjust = xInch - kai.deadwheels.currentX;
            yAdjust = yInch - kai.deadwheels.currentY;
        }

        return false;
    }

    // Returns true on robot detected
    private boolean checkPathing() {
        kai.deadwheels.wheelLoop();

        double robotAngle = kai.getHeading();

        for(int i = 0; i < 4; i++) {
            double distance = robotSensors[i].getDistance(DistanceUnit.INCH);
            double[] sensorOffset = sensorOffsets[i];

            double xIntercept = Math.cos(robotAngle + sensorOffset[3]) * distance + kai.deadwheels.currentX;
            double yIntercept = Math.sin(robotAngle + sensorOffset[3]) * distance + kai.deadwheels.currentY;

            double xTile = xIntercept / 24;
            double yTile = 6 - (yIntercept / 24);

            // If the intercept is in the field (with 6 inches of margin), mark that tile as blocked
            // Also check if the distance is out of range and ignore it if it is
            if(xIntercept < 132 && xIntercept > 12 && yIntercept < 132 && yIntercept > 12 && distance <= 60) {
                int nodeIndex = ((int) Math.floor(yTile)) * 6 + ((int) Math.floor(xTile));

                int robotStartIndex = (int)(kai.deadwheels.currentY / 24) * 6 + (int)(kai.deadwheels.currentX / 24);
                dStar.updateStart(robotStartIndex);
                dStar.markBlocked(nodeIndex);
                return true;
            } else {
                for(Integer nodeIndex : dStar.obstacles) {
                    double obstacleX = nodeIndex % 6;
                    double obstacleY = Math.floor(nodeIndex / 6f);

                    double robotXTile = kai.deadwheels.currentX / 24;
                    double robotYTile = 6 - (kai.deadwheels.currentY / 24);

                    // Get minimum distance between sensor line and tiles marked as obstacles
                    double xOffset = xTile - robotXTile;
                    double yOffset = yTile - robotYTile;

                    double endObstacleXOffset = obstacleX - xTile;
                    double endObstacleYOffset = obstacleY - yTile;

                    double startObstacleXOffset = obstacleX - robotXTile;
                    double startObstacleYOffset = obstacleY - robotYTile;

                    double distanceB = (xOffset * endObstacleXOffset + yOffset * endObstacleYOffset);
                    double distanceA = (xOffset * startObstacleXOffset + yOffset * startObstacleYOffset);

                    double finalDistance = 0;

                    if(distanceB < 0) {
                        finalDistance = Math.hypot(obstacleY - yTile, obstacleX - xTile);
                    } else if(distanceA < 0) {
                        finalDistance = Math.hypot(obstacleY - robotYTile, obstacleX - robotXTile);
                    } else {
                        double mod = Math.hypot(xOffset, yOffset);
                        finalDistance = Math.abs(xOffset * startObstacleYOffset - yOffset * startObstacleXOffset) / mod;
                    }

                    if(finalDistance < 6) {
                        dStar.markOpen(nodeIndex);
                    }
                }
            }
        }
        return false;
    }
}
