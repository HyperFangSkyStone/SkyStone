package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;


@TeleOp(name="LiftandInTakeTeleOp", group="Pushbot")
//@Disabled
public class LiftandIntakeTeleOp extends LinearOpMode {

    LiftandIntakeHW Ref = new LiftandIntakeHW();
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Ref.init(hardwareMap);

        waitForStart();

        double power1 = gamepad1.right_trigger;
        double power2 = gamepad1.left_trigger;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                Ref.IntakeM1.setPower(1);
                Ref.IntakeM2.setPower(1);
            } else {
                Ref.IntakeM1.setPower(0);
                Ref.IntakeM2.setPower(0);
            }

            if (Math.abs(power1) > 0.1) {
                Ref.LiftM1.setPower(power1);
                Ref.LiftM2.setPower(power1);
            } else if (Math.abs(power2) > 0.1) {
                Ref.LiftM1.setPower(-power2);
                Ref.LiftM2.setPower(-power2);
            } else {
                Ref.LiftM1.setPower(0);
                Ref.LiftM2.setPower(0);
            }

        }

    }

}

