package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class sets up and holds methods for using the goal lift mechanism
 */
public class GoalLift {

    private DcMotor motor;

    private final int TICKS_TO_LIFTED_POSITION = 1440;

    public enum LiftPosition{
        TOP,
        BOTTOM
    }

    private LiftPosition currentPosition;

    /**
     * Creates a GoalLift with the default name for the motor
     * @param hw robot's hardware map
     */
    public GoalLift(HardwareMap hw){
        motor = hw.dcMotor.get("goalLiftMotor");
    }

    /**
     * Creates a GoalLift with a specified name for the motor
     * @param hw robot's hardware map
     * @param motorName name of the lift motor in the hardware map
     */
    public GoalLift(HardwareMap hw, String motorName){
        motor = hw.dcMotor.get(motorName);
    }

    /**
     * Sets power to the goal lift
     * @param power power at which to run the motor
     */
    public void setGoalLiftPower(double power){
        motor.setPower(power);
    }

    /**
     * Moves the lift to the top or bottom
     * @param position position to move the lift to
     * @param power power at which to move the lift
     */
    public void setGoalLiftPosition(LiftPosition position, double power){
        switch(position){
            case TOP: {
                if (currentPosition == LiftPosition.TOP) {
                    break;
                }
                liftToPosition(TICKS_TO_LIFTED_POSITION, power);
            }
            case BOTTOM:{
                if(currentPosition == LiftPosition.BOTTOM){
                    break;
                }
                liftToPosition(0, power);
            }
        }

    }

    /**
     * Runs lift to specified position
     * @param position position to move the lift to
     * @param power power at which to move the lift
     */
    private void liftToPosition(int position, double power){
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(position);
        setGoalLiftPower(power);
        while(motor.isBusy());
        setGoalLiftPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
