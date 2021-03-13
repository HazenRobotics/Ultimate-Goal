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
public class GoalLift {

    TouchSensor liftedButton;
    TouchSensor loweredButton;

    Servo claw;

    private double openClawPosition;
    private double closedClawPosition;

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

    private ClawPosition currentClawPosition;

    /**
     * Creates a GoalLift with the default name for the motor
     * @param hw robot's hardware map
     */
    public GoalLift(HardwareMap hw, double openPosition, double closedPosition ){
        this(hw, "goalLift", "claw", "liftedButton", "loweredButton", openPosition, closedPosition );
    }

    /**
     * Creates a GoalLift with a specified name for the motor
     * @param hw robot's hardware map
     * @param motorName name of the lift motor in the hardware map
     */
    public GoalLift(HardwareMap hw, String motorName, String clawName, String liftedButtonName, String loweredButtonName, double openPosition, double closedPosition ) {

        lift = hw.dcMotor.get( motorName );

        claw = hw.servo.get( clawName );

        this.openClawPosition = openPosition;
        this.closedClawPosition = closedPosition;

        liftedButton = hw.touchSensor.get(liftedButtonName);
        loweredButton = hw.touchSensor.get(loweredButtonName);

        //change this based on needed motor direction
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
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

                currentLiftPosition = LiftPosition.LIFTED;
                liftToPosition( LiftPosition.LIFTED, power );
                break;
            }
            case LOWERED:{
                if( currentLiftPosition == LiftPosition.LOWERED )
                    break;

                currentLiftPosition = LiftPosition.LOWERED;
                liftToPosition( LiftPosition.LOWERED, power );
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
                currentClawPosition = ClawPosition.CLOSED;
                claw.setPosition(closedClawPosition);
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

    public void setCurrentLiftPosition( LiftPosition liftPosition ) {
        currentLiftPosition = liftPosition;
    }

    /**
     *
     * @return the claw's position as a variable of double
     */
    public double getClawPosition() {
        return claw.getPosition();
    }

    /**
     *
     * @return the lift's power
     */
    public double getLiftPower() {
        return lift.getPower();
    }

    /**
     *
     * @return the claw's location as a variable of ClawPosition
     */
    public ClawPosition getClawLocation() {
        return currentClawPosition;
    }

    /**
     *
     * @return the lift's location as a variable of LiftPosition
     */
    public LiftPosition getCurrentLiftPosition() {
        return currentLiftPosition;
    }
}
