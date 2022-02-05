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
        boolean onRed = fieldSide == FieldSide.RED ? true : false;

        waitForStart();

        hueTrackingPipeline.startVideo();

    // DETECT FIELD ORIENTATION
        armControl(ArmPosition.UP); // Raise arm for camera to see
        if(protectedSleep(1000)) return;

        fieldOrientation = getFieldOrientation(fieldSide); // See

        telemetry.addData("Target", fieldOrientation.toString());
        telemetry.update();

    // PLACE PRELOAD
        driveDist(4);

        startTurnTo(onRed ? 4*Math.PI/6 : 2*Math.PI/6); // Turn to face the shipping hub
        while(turningFlag) updateTurnTo();

        driveToShippingHub(fieldSide); // Go to the shipping hub

        drive(-.5, -.5);

        switch(fieldOrientation) {
            case HIGH_FORE:
                if(protectedSleep(300)) return;
                break;
            case MED_FORE:
                if(protectedSleep(400)) return;
                break;
            case LOW_FORE:
                if(protectedSleep(400)) return;
                break;
        }

        drive(0, 0);

        armControl(fieldOrientation);
        if(protectedSleep(500)) return; // Eject the block on the right height
        runIntake(.6);
        if(protectedSleep(1000)) return;

    // MOVE FROM HUB
        armControl(ArmPosition.UP); // Raise the arm
        runIntake(0);
        if(protectedSleep(400)) return;

        drive(-.7, -.7); // Back up
        if(protectedSleep(400)) return;
        drive(0, 0);

        /*
    // GO TO FREIGHT
        startTurnTo(onRed ? -0.15 : Math.PI+0.15); // Face the freight
        while(turningFlag) updateTurnTo();

        armControl(ArmPosition.HIGH_FORE); // Lower arm a bit

        driveToFreight(); // Go to the freight

        armControl(ArmPosition.PICKUP); // Place the arm down
        if(protectedSleep(500)) return;

    // GRAB FREIGHT
        runIntake(-1); // Start intake

        drive(-1, -1); // Jiggle to grab freight
        if(protectedSleep(300)) return;
        drive(1, 1);
        if(protectedSleep(1000)) return;
        drive(0, 0);

        runIntake(0); // Stop intake

    // RETURN TO SHIPPING HUB
        armControl(ArmPosition.HIGH_FORE); // Raise arm

        drive(-1, -1); // back up
        if(protectedSleep(400)) return;
        drive(0, 0);

        startTurnTo(onRed ? Math.PI-0.15 : 0.15); // Turn to face shipping hub
        while(turningFlag) updateTurnTo();

        driveToShippingHub(fieldSide); // Navigate to the shipping hub
        drive(-.5, -.5);
        if(protectedSleep(300)) return;

        runIntake(0.6);

    // MOVE FROM HUB
        armControl(ArmPosition.UP); // Raise the arm
        runIntake(0);
        if(protectedSleep(400)) return;

        drive(-.7, -.7); // Back up
        if(protectedSleep(400)) return;
        drive(0, 0);*/

    // PARK
        startTurnTo(onRed ? -0.15 : Math.PI+0.15); // Face the freight
        while(turningFlag) updateTurnTo();

        armControl(ArmPosition.HIGH_FORE); // Lower arm a bit

        driveToFreight(); // Go to the freight

        armControl(ArmPosition.PICKUP); // Place the arm down
        if(protectedSleep(500)) return;

        stopOpenCV();
    }
}
