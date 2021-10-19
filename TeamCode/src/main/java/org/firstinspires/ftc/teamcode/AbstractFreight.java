package org.firstinspires.ftc.teamcode;

public abstract class AbstractFreight extends AbstractAutonomous {
    protected ArmPosition fieldOrientation;

    public void run(FieldSide fieldSide) {
        // TODO: Use fieldSide
        boolean onRed = fieldSide == FieldSide.RED;

        initializeValues();

        waitForStart();

        fieldOrientation = getFieldOrientation();

        driveDist(6); // Get away from the wall

        startTurnTo(onRed ? 3*Math.PI/4 : Math.PI/4);
        while(turningFlag) updateTurnTo(); // Face the goal

        driveDist(16); // Drive to the goal

        armControl(fieldOrientation); // Position the arm

        startTurnTo(onRed ? -Math.PI/4 : 5*Math.PI/4); // Align the arm with the goal
        while(turningFlag) updateTurnTo();

        setFlup(true); // Dump preload
        runIntake(true);
        sleep(200);

        setFlup(false); // Stop intake
        runIntake(false);
        armControl(ArmPosition.PICKUP); // Prepare arm for pickup

        startTurnTo(onRed ? Math.PI : 0); // Face the freight
        while(turningFlag) updateTurnTo();

        driveDist(20); // Drive to the barrier

        drive(0.7, 0.7); // Drive over the barrier
        sleep(500);
        drive(0, 0);
        sleep(200);

        startTurnTo(onRed ? Math.PI : 0);
        while(turningFlag) updateTurnTo(); // Realign after crossing the barrier

        runIntake(true); // Attempt to grab freight
        drive(0.4, 0.4);
        sleep(700);
        // TODO: Branching program to detect failure
        drive(0, 0); // Stop attempting to grab freight
        runIntake(false);
        sleep(200);

        startTurnTo(onRed ? 7*Math.PI/6 : -Math.PI/6);
        while(turningFlag) updateTurnTo(); // Face back towards goal

        driveDist(-24); // Drive to goal

        armControl(ArmPosition.HIGH); // Position arm to high goal

        startTurnTo(onRed ? 5*Math.PI/4 : -Math.PI/4); // Align arm with the goal
        while(turningFlag) updateTurnTo();

        setFlup(true); // Dump freight
        runIntake(true);
        sleep(200);

        setFlup(false); // Stop intake
        runIntake(false);
        armControl(ArmPosition.START);

        startTurnTo(onRed ? 7*Math.PI/6 : -Math.PI/6); // Face the barrier
        while(turningFlag) updateTurnTo();

        driveDist(24); // Park
    }
}
