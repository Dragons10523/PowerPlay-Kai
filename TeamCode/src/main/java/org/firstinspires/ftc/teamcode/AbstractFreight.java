package org.firstinspires.ftc.teamcode;

/* CLASS SUMMARY:
 * Drives the autonomous path used for the freight side, toggle to change between red and blue
 *  */

// ORDER OF OPERATIONS: Preload -> Cycle -> Park

public abstract class AbstractFreight extends AbstractBarcode {
    protected ArmPosition fieldOrientation;

    public void run(FieldSide fieldSide) {
        initializeValues();
        startOpenCV();

        waitForStart();

        armControl(ArmPosition.UP);

        if(protectedSleep(300)) return;

        fieldOrientation = getFieldOrientation();

        boolean onRed = fieldSide == FieldSide.RED ? true : false;

        telemetry.addLine("Target arm pos: " + fieldOrientation.toString());
        telemetry.addLine("Color: " + fieldSide.toString());
        telemetry.update();

        drive(1, 1);
        if(protectedSleep(500));
        drive(0, 0);

        startTurnTo(onRed ? Math.PI/4 : 3*Math.PI/4);

        driveToShippingHub(fieldSide); // Place the preload box

        armControl(fieldOrientation);
        if(protectedSleep(500)) return;
        runIntake(1);
        if(protectedSleep(1000)) return;

        armControl(ArmPosition.UP);
        runIntake(0);

        /*

        while(getRuntime() < 20) {
            armControl(ArmPosition.UP);

            drive(-1, -1); // Back up
            if(protectedSleep(500)) return;
            drive(0, 0);

            startTurnTo(onRed ? Math.PI : 0); // Face the freight
            while(turningFlag) updateTurnTo();
            if(isStopRequested()) return; // Safety

            driveToFreight(); // Go to the freight
            if(isStopRequested()) return; // Safety

            armControl(ArmPosition.LOW_FORE);
            if(protectedSleep(150)) return;
            armControl(ArmPosition.PICKUP);
            if(protectedSleep(150)) return;

            drive(0.7, 0.7); // Grab the freight
            runIntake(-1);
            if(protectedSleep(500)) return;
            drive(-1, -1);
            runIntake(0);
            if(protectedSleep(100)) return;
            drive(0, 0);

            startTurnTo(onRed ? 0 : Math.PI); // Face the shiping hub
            while(turningFlag) updateTurnTo();
            if(isStopRequested()) return; // Safety

            driveToShippingHub(fieldSide); // Drive to the shipping hub
            if(isStopRequested()) return; // Safety
            armControl(ArmPosition.HIGH_FORE);
            while(ahi.arm.isBusy() && opModeIsActive()) sleep(50);
            runIntake(0.5);
            if(protectedSleep(500)) return;
            runIntake(0);
        }*/

        drive(-1, -1); // Back up
        if(protectedSleep(500)) return;
        drive(0, 0);

        startTurnTo(onRed ? Math.PI : 0); // Face the freight
        while(turningFlag) updateTurnTo();

        driveToFreight();

        armControl(ArmPosition.LOW_FORE);
        if(protectedSleep(500)) return;

        stopOpenCV();
    }
}
