package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;

public class MecanumDriveSubsystems extends SubsystemBase {

    public MecanumDriveSubsystems(){

    }
    public void driveWithVector(Vector2d vector) {
        double[] speeds = normalize(new double[]{vector.getX(), vector.getY()});
        mecanum.driveRobotCentric(speeds[0], speeds[1],0);
    }

    public double[] normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }

        return wheelSpeeds;
    }
}
