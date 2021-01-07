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

    private  ClawPosition currentClawPosition;

    /**
     * Creates a GoalLift with the default name for the motor
     * @param hw robot's hardware map
     */
    public GoalLift( HardwareMap hw ){
        this(hw, "goalLiftMotor", "claw", "liftedButton", "loweredButton");
    }

    /**
     * Creates a GoalLift with a specified name for the motor
     * @param hw robot's hardware map
     * @param motorName name of the lift motor in the hardware map
     */
    public GoalLift( HardwareMap hw, String motorName, String clawName, String liftedButtonName, String loweredButtonName ) {
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

                liftToPosition( TICKS_TO_LIFTED_POSITION, power );
                currentLiftPosition = LiftPosition.LIFTED;
                break;
            }
            case LOWERED:{
                if( currentLiftPosition == LiftPosition.LOWERED)
                    break;

                liftToPosition( 0, power );
                currentLiftPosition = LiftPosition.LOWERED;
                break;
            }
        }

    }
    public void setClawPosition( ClawPosition position) {
        switch (position) {
            case OPEN:
                claw.setPosition(1.0);
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
    private void liftToPosition( int position, double power ) {

        motor.setMode( DcMotor.RunMode.RUN_TO_POSITION );
        motor.setTargetPosition( position );
        setGoalLiftPower( power );
        while( motor.isBusy() );
            setGoalLiftPower( 0 );
        motor.setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER );
    }

    public ClawPosition getCurrentClawPosition() {
        return currentClawPosition;
    }

    public LiftPosition getCurrentLiftPosition() {
        return currentLiftPosition;
    }
}
