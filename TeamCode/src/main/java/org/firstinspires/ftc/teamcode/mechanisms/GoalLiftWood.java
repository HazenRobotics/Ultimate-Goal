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
public class    GoalLiftWood {

    TouchSensor liftedButton;
    TouchSensor loweredButton;

    private Servo claw;

    private DcMotor motor;

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
    public GoalLiftWood( HardwareMap hw ){
        this(hw, "goalLift", "claw", "liftedButton", "loweredButton");
    }

    /**
     * Creates a GoalLift with a specified name for the motor
     * @param hw robot's hardware map
     * @param motorName name of the lift motor in the hardware map
     */
    public GoalLiftWood( HardwareMap hw, String motorName, String clawName, String liftedButtonName, String loweredButtonName ) {
        claw = hw.servo.get(clawName);

        motor = hw.dcMotor.get( motorName );

        liftedButton = hw.touchSensor.get(liftedButtonName);
        loweredButton = hw.touchSensor.get(loweredButtonName);

        //change this based on needed motor direction
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    /**
     * Sets power to the goal lift
     * @param power power at which to run the motor
     */
    public void setGoalLiftPower( double power ) {

        motor.setPower( power );
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
                claw.setPosition(0.5);
                currentClawPosition = ClawPosition.OPEN;
                break;
            case CLOSED:
                claw.setPosition(0.0);
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
        if(position == LiftPosition.LIFTED) {
            motor.setPower(power);
            while (!liftedButton.isPressed());
            motor.setPower(0);
        }
        else {
            motor.setPower(-power);
            while (!loweredButton.isPressed());
            motor.setPower(0);
        }

    }

    public ClawPosition getCurrentClawPosition() {
        return currentClawPosition;
    }

    public LiftPosition getCurrentLiftPosition() {
        return currentLiftPosition;
    }

    public double getClawPosition() {
        return claw.getPosition();
    }

    public double getLiftPower() {
        return motor.getPower();
    }
}
