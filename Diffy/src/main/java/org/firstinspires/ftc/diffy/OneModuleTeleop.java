package org.firstinspires.ftc.diffy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp(name="OneModuleTeleop", group="1")
//@Disabled
public class OneModuleTeleop extends LinearOpMode {

    OneModuleHardware omw = new OneModuleHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        omw.init(hardwareMap);

        double prop = 0.15;
        waitForStart();
        // test
        //double holdturny = 0;
        while (opModeIsActive()) {


            double throttle = gamepad1.right_trigger;
            double anglediff = 0;
            double turny = 0;
            if (Math.abs(gamepad1.right_stick_x) + Math.abs(gamepad1.right_stick_y) > 0.2) {
                turny = Math.atan2(-gamepad1.right_stick_x, -gamepad1.right_stick_y);
            }

            anglediff = omw.getAngle() - turny;
            if (anglediff > Math.PI)
                anglediff -= 2 * Math.PI;
            if (anglediff < -Math.PI)
                anglediff += 2 * Math.PI;

            if (Math.abs(anglediff) < .05) {
                omw.LeftM1.setPower(throttle);
                omw.RightM1.setPower(-throttle);
            } else {
                omw.LeftM1.setPower(throttle + anglediff * prop);
                omw.RightM1.setPower(-throttle + anglediff * prop);
            }


            telemetry.addData("Encoder Voltage: ", omw.EncoderM.getVoltage());
            telemetry.addData("Encoder Angle: ", omw.getAngle());
            telemetry.addData("Turny: ", turny);
            //telemetry.addData("HoldTurny: ", holdturny);
            telemetry.addData("Angle Diff: ", anglediff);

            telemetry.update();

        }

        omw.freeze();
    }

}
