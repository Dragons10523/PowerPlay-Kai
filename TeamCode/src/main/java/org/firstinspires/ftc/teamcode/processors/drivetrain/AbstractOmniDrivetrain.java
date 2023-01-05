package org.firstinspires.ftc.teamcode.processors.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.processors.VecUtils;

public abstract class AbstractOmniDrivetrain extends AbstractDrivetrain {
    double impulseRotation;
    final static double ROOT2OVER2 = Math.sqrt(2) / 2;

    public AbstractOmniDrivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, double impulseRotation) {
        super(frontLeft, frontRight, backLeft, backRight);
        this.impulseRotation = impulseRotation;
    }

    public void drive(float forward, float sideway, double turn) {
        VectorF powerVector = new VectorF(forward, sideway);
        VecUtils.rotateVector(powerVector, impulseRotation);

        float negativeGroup = (float) (powerVector.get(0) / ROOT2OVER2);
        float positiveGroup = (float) (powerVector.get(1) / ROOT2OVER2);

        driveMotors[0].setPower(negativeGroup + turn);
        driveMotors[1].setPower(positiveGroup - turn);
        driveMotors[2].setPower(positiveGroup + turn);
        driveMotors[3].setPower(negativeGroup - turn);
    }
}
