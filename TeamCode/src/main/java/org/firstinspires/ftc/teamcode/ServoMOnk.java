package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Servo MONKEY", group="Capuchin")
//@Disabled
public class ServoMOnk extends LinearOpMode {

    TankDriveALPHA servo = new TankDriveALPHA();

    @Override
    public void runOpMode() throws InterruptedException
    {
        servo.init(hardwareMap);

        waitForStart();


        while(opModeIsActive())
        {

            if (gamepad1.a)
            {
                servo.LServo.setPosition(1);
                servo.RServo.setPosition(0);
            }

            else if (gamepad1.x)
            {
                servo.LServo.setPosition(0);
                servo.RServo.setPosition(1);
            }
        }

    }


}
