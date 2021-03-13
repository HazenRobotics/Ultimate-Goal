package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * This class sets up and holds methods for using the goal lift mechanism
 * The goal lift mechanism is the ______ that does _______
 */
public class GoalLiftWood {

    TouchSensor liftedButton;
    TouchSensor loweredButton;

    Servo claw;

    private static double openClawPosition;
    private static double closedClawPosition;

    private DcMotor lift;

    private final int TICKS_TO_LIFTED_POSITION = 1440;

    public enum LiftPosition {
        LIFTED,
        LOWERED
    }
    public enum ClawPosition {
        OPEN,
        CLOSED
    }

    private LiftPosition currentLiftPosition;

    private  ClawPosition currentClawPosition;

    /**
     * Creates a GoalLift with the default name for the motor
     * @param hw robot's hardware map
     */
    public GoalLiftWood(HardwareMap hw, double openPosition, double closedPosition ){
        this(hw, "goalLift", "claw", "liftedButton", "loweredButton", openPosition, closedPosition );
    }

    /**
     * Creates a GoalLift with a specified name for the motor
     * @param hw robot's hardware map
     * @param motorName name of the lift motor in the hardware map
     */
    public GoalLiftWood(HardwareMap hw, String motorName, String clawName, String liftedButtonName, String loweredButtonName, double openPosition, double closedPosition ) {

        lift = hw.dcMotor.get( motorName );

        claw = hw.servo.get(clawName);

        this.openClawPosition = openPosition;
        this.closedClawPosition = closedPosition;

        liftedButton = hw.touchSensor.get(liftedButtonName);
        loweredButton = hw.touchSensor.get(loweredButtonName);

        //change this based on needed motor direction
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Sets power to the goal lift
     * @param power power at which to run the motor
     */
    public void setGoalLiftPower( double power ) {

        lift.setPower( power );
    }

    /**
     * Moves the lift to the top or bottom
     * @param position position to move the lift to
     * @param power power at which to move the lift
     */
    public void setGoalLiftPosition( LiftPosition position, double power ) {

        switch( position ) {
            case LIFTED: {
                if ( currentLiftPosition == LiftPosition.LIFTED)
                    break;

                liftToPosition( LiftPosition.LIFTED, power );
                currentLiftPosition = LiftPosition.LIFTED;
                break;
            }
            case LOWERED:{
                if( currentLiftPosition == LiftPosition.LOWERED)
                    break;

                liftToPosition( LiftPosition.LOWERED, power );
                currentLiftPosition = LiftPosition.LOWERED;
                break;
            }
        }

    }
    public void setClawPosition( ClawPosition position) {
        switch (position) {
            case OPEN:
                claw.setPosition(openClawPosition);
                currentClawPosition = ClawPosition.OPEN;
                break;
            case CLOSED:
                claw.setPosition(closedClawPosition);
                currentClawPosition = ClawPosition.CLOSED;
                break;
        }
    }

    /**
     * Runs lift to specified position
     * @param position position to move the lift to
     * @param power power at which to move the lift
     */
    private void liftToPosition( LiftPosition position, double power ) {
        long time = System.currentTimeMillis() + 1000;
        if(position == LiftPosition.LIFTED) {
            lift.setPower(power);
            while (!liftedButton.isPressed() && System.currentTimeMillis() < time);
        }
        else {
            lift.setPower(-power);
            while (!loweredButton.isPressed() && System.currentTimeMillis() < time);
        }
        lift.setPower(0);
    }

    public double getClawPosition() {
        return claw.getPosition();
    }

    public double getLiftPower() {
        return lift.getPower();
    }

    public ClawPosition getCurrentClawPosition() {
        return currentClawPosition;
    }

    public LiftPosition getCurrentLiftPosition() {
        return currentLiftPosition;
    }
}
