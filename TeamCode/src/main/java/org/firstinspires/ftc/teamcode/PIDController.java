package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double kP, kI, kD;
    private double errorPrev, timePrev;

    public PIDController(double kP)
    {
        this.kP = kP;
        this.kI = 0;
        this.kD = 0;

        errorPrev = 0;
        timePrev = 0;
    }

    public PIDController(double kP, double kD)
    {
        this.kP = kP;
        this.kI = 0;
        this.kD = kD;

        errorPrev = 0;
        timePrev = 0;
    }

    public PIDController(double kP, double kI, double kD)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        errorPrev = 0;
        timePrev = 0;
    }

    public void reset()
    {
        errorPrev = 0;
        timePrev = 0;
    }

    public double PIDOutput(double target, double current, double time, double floor)
    {
        double error = target - current;
        double p = error / target * kP;
        double d = ((error - errorPrev) / (time - timePrev)) /target * kD;
        double i = error * (time - timePrev) * kI;

        double output = p + i + d;

        if (output < 0 && output > -floor)
            output = -floor;
        if (output > 0 && output < floor)
            output = floor;


        errorPrev = error;
        timePrev = time;


        return output;
    }
}

