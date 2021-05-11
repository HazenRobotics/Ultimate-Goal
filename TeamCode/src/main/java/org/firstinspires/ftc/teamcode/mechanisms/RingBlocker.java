package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.ShootingMath;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;

/**
 * This class sets up and holds methods for using the ring shooter mechanism
 * The ring shooter mechanism is the ______ that does _______
 */
public class RingBlocker {

    public Servo blocker;

    private double blockedPosition;
    private double retractedPosition;

    private static final double RING_PUSH_TIME = 800; // milliseconds
    private static final double RING_RETRACT_TIME = 800;

    public enum BlockerPosition {
        BLOCKED,
        RETRACTED
    }

    private BlockerPosition blockerPosition;

    public RingBlocker(HardwareMap hw, double blockedPosition, double retractedPosition ) {

        this( hw, "blocker", blockedPosition, retractedPosition );
    }

    public RingBlocker(HardwareMap hw, String blockerName, double blockedPosition, double retractedPosition ) {

        blocker = hw.servo.get(blockerName);

        this.blockedPosition = blockedPosition;
        this.retractedPosition = retractedPosition;
    }

    public void setNumericalBlockerPosition( double position ) {
        blocker.setPosition( position );
    }

    public void setBlockerPosition( BlockerPosition position ) {
        switch (position) {
            case BLOCKED:
                blockerPosition = BlockerPosition.BLOCKED;
                setNumericalBlockerPosition(blockedPosition);
                break;
            case RETRACTED:
                blockerPosition = BlockerPosition.RETRACTED;
                setNumericalBlockerPosition(retractedPosition);
                break;
        }
    }

    public void setNumericalBlockerPositionAsync( double position ) {
        new Thread(() -> setNumericalBlockerPosition( position )).start();
    }

    public void setBlockerPositionAsync( BlockerPosition position ) {
        new Thread(() -> setBlockerPosition( position )).start();
    }

    public double getBlockerPosition() {
        return blocker.getPosition();
    }

    public BlockerPosition getPusherLocation() {
        return blockerPosition;
    }


}