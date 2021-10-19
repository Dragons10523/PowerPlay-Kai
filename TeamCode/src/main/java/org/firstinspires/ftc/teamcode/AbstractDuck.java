package org.firstinspires.ftc.teamcode;

/* CLASS SUMMARY:
 * Drives the autonomous path used for the duck side, toggle to change between red and blue
 *  */

// ORDER OF OPERATIONS: Ducks -> Preload -> Cycle -> Park

public abstract class AbstractDuck extends AbstractAutonomous {
    protected FieldOrientation fieldOrientation;

    public void run(FieldSide fieldSide) {
        // TODO: Use fieldSide

        initializeValues();

        waitForStart();

        fieldOrientation = getFieldOrientation();

        driveDist(3); // Get away from the wall

        startTurnTo(Math.PI);
        while(turningFlag) updateTurnTo(); // Turn to face the ducks


    }
}
