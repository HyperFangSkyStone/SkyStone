package org.firstinspires.ftc.mercury.DISABLED;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.mercury.DISABLED.OneMotorHardware;

@TeleOp(name="OneMotor TeleOp", group="Capuchin")
@Disabled
public class OneMotorTeleopTest extends LinearOpMode {

    OneMotorHardware oneMotor = new OneMotorHardware();

    @Override
    public void runOpMode() throws InterruptedException
    {
        oneMotor.init(hardwareMap);

        waitForStart();


        while(opModeIsActive())
        {

            telemetry.addData("Encoder Position", oneMotor.LM0.getCurrentPosition());
            telemetry.update();
        }

    }


}
