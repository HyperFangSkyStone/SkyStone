package org.firstinspires.ftc.teamcode.EncoderTest;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderHW {

    public DcMotor M0 = null;
    public DcMotor M1 = null;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public EncoderHW() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        M0 = hwMap.get(DcMotor.class, "M0");
        M1 = hwMap.get(DcMotor.class, "M1");
        M0.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        M1.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        M0.setPower(0);
        M1.setPower(0);

        M0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        M0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
