package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Mushu;
import org.firstinspires.ftc.teamcode.commands.DriveWithSticks;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveWithSticks;

public class Drive extends CommandOpMode {
    Mushu mushu;

    @Override
    public void initialize() {
        mushu = Mushu.GetInstance(this);

        schedule(new MecanumDriveWithSticks(mushu.mecanum, mushu.driverGamepad));

    }
}
