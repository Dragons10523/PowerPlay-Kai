package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.commands.DriveWithSticks;

public class Drive extends CommandOpMode {
    Mushu mushu;

    @Override
    public void initialize() {
        mushu = Mushu.GetInstance(this);

        schedule(new DriveWithSticks(mushu.drivetrain, mushu.driverGamepad));
    }
}
