package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class GeneralPID {

    PIDCoefficients coefficients;

    double previousTime;
    double previousError;

    double integral;

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
        double derivative = (error - previousError) / time; // just error / time
        previousError = error; // set previousError to current error
        previousTime = System.currentTimeMillis(); // set previousTime to current time
        return coefficients.p * error + coefficients.i * this.integral + coefficients.d * derivative;
    }

    public void reset() {
        integral = 0;
        previousTime =0;
        previousError = 0;
    }




}
