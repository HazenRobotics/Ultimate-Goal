package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

public class RobotClapper extends Robot {

    public RobotClapper(HardwareMap hw){
        super(hw);
        drive = new MecanumDrive(hw);
    }

}
