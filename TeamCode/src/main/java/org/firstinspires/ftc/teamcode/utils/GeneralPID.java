package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.robots.Robot;

public class GeneralPID {

    PIDCoefficients coefficients;

    double previousTime;
    double previousError;

    double integral;

    final static double minPow = 0.3;
    final static double maxPow = 1.0;

    public GeneralPID( PIDCoefficients PIDCoeff ) {
        coefficients = PIDCoeff;
    }

    /**
     * target & curInput should be in the same unit
     * @param target target variable
     * @param curInput current input variable
     * @return PID correction values
     */
    public double getPID( double target, double curInput ) {

        if( previousTime == 0 ) previousTime = System.currentTimeMillis();
        double time = System.currentTimeMillis() - previousTime;
        double error = target - curInput; // Error = Target - Actual
        integral += (error * time); // Integral is increased by the error*time
        double derivative = time == 0 ? 0 : (error - previousError) / time; // just error / time

        String info = "";

        info += "getPID( target, curInput) :: getPID( " + target + ", " + curInput + " )" + System.lineSeparator();
        info += "time :: " + time + System.lineSeparator();
        info += "error :: " + error + System.lineSeparator();
        info += "integral :: " + integral + System.lineSeparator();
        info += "derivative :: " + derivative + System.lineSeparator();
        info += "previousError :: " + previousError + System.lineSeparator();
        info += "previousTime :: " + previousTime + System.lineSeparator();
        info += "return :: " + (coefficients.p * error + coefficients.i * this.integral + coefficients.d * derivative) + System.lineSeparator();
        info += "-----------------------------------------------" + System.lineSeparator();


        if( target == 15 ) Robot.writeToDefaultFile( info, true, false );

        previousError = error; // set previousError to current error
        previousTime = System.currentTimeMillis(); // set previousTime to current time

        /*
        // NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
        double power = (((OldValue - OldMin) * (maxPow - minPow)) / (OldMax - OldMin)) + minPow;
        */
        return coefficients.p * error + coefficients.i * this.integral + coefficients.d * derivative;
    }

    public void reset() {
        integral = 0;
        previousTime =0;
        previousError = 0;
    }




}
