package org.firstinspires.ftc.mercury;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class TankDriveHardware
{

    public DcMotor  LM0  = null;
    public DcMotor  LM1  = null;
    public DcMotor  RM0  = null;
    public DcMotor  RM1  = null;


    HardwareMap hwMap =  null;

    public TankDriveHardware(){
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        LM0 = hwMap.get(DcMotor.class, "LM0");
        LM1 = hwMap.get(DcMotor.class, "LM1");
        RM0 = hwMap.get(DcMotor.class, "RM0");
        RM1 = hwMap.get(DcMotor.class, "RM1");


        LM0.setDirection(DcMotor.Direction.REVERSE);
        LM1.setDirection(DcMotor.Direction.REVERSE);
        RM0.setDirection(DcMotor.Direction.FORWARD);
        RM1.setDirection(DcMotor.Direction.FORWARD);


        LM0.setPower(0);
        LM1.setPower(0);
        RM0.setPower(0);
        RM1.setPower(0);


        LM0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RM0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        LM0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RM0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LM0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RM0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public double getAverageEncoder(char c) {
        switch (c)
        {
            case 'l':
                if (LM0.getCurrentPosition() == 0)
                    return LM1.getCurrentPosition();
                else if (LM1.getCurrentPosition() == 0)
                    return LM0.getCurrentPosition();
                else return (Math.abs(LM0.getCurrentPosition()) + Math.abs(LM1.getCurrentPosition())) / 2.0;
            case 'r':
                if (RM0.getCurrentPosition() == 0)
                    return RM1.getCurrentPosition();
                else if (RM1.getCurrentPosition() == 0)
                    return RM0.getCurrentPosition();
                else return (Math.abs(RM0.getCurrentPosition()) + Math.abs(RM1.getCurrentPosition())) / 2.0;
        }
        return 0 / 0;
    }

    public void resetEncoders() {

        LM0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RM0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RM1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LM0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RM0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

