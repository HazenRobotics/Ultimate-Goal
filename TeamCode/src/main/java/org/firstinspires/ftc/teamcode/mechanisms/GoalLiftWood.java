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

    Servo claw;

    private DcMotor motor;

    public enum ClawPosition {
        OPEN,
        CLOSED
    }

    private ClawPosition currentClawPosition;

    /**
     * Creates a GoalLift with the default name for the motor
     * @param hw robot's hardware map
     */
    public GoalLiftWood( HardwareMap hw ){
        this(hw, "goalLift", "claw");
    }

    /**
     * Creates a GoalLift with a specified name for the motor
     * @param hw robot's hardware map
     * @param motorName name of the lift motor in the hardware map
     */
    public GoalLiftWood( HardwareMap hw, String motorName, String clawName) {
        claw = hw.servo.get(clawName);

        motor = hw.dcMotor.get( motorName );

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

    public ClawPosition getCurrentClawPosition() {
        return currentClawPosition;
    }
}
