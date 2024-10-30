package org.firstinspires.ftc.teamcode.previous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ControllerTest", group="Test")
public class ControllerTestPrevious extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Gamepad 1
            telemetry.addLine("Gamepad 1:");
            telemetry.addData("Left Stick X", String.format("%.4f", gamepad1.left_stick_x));
            telemetry.addData("Left Stick Y", String.format("%.4f", gamepad1.left_stick_y));
            telemetry.addData("Right Stick X", String.format("%.4f", gamepad1.right_stick_x));
            telemetry.addData("Right Stick Y", String.format("%.4f", gamepad1.right_stick_y));
            telemetry.addData("DPad Up", gamepad1.dpad_up);
            telemetry.addData("DPad Down", gamepad1.dpad_down);
            telemetry.addData("DPad Left", gamepad1.dpad_left);
            telemetry.addData("DPad Right", gamepad1.dpad_right);
            telemetry.addData("A", gamepad1.a);
            telemetry.addData("B", gamepad1.b);
            telemetry.addData("X", gamepad1.x);
            telemetry.addData("Y", gamepad1.y);
            telemetry.addData("Left Bumper", gamepad1.left_bumper);
            telemetry.addData("Right Bumper", gamepad1.right_bumper);
            telemetry.addData("Left Trigger", String.format("%.2f", gamepad1.left_trigger));
            telemetry.addData("Right Trigger", String.format("%.2f", gamepad1.right_trigger));
            telemetry.addData("Back", gamepad1.back);
            telemetry.addData("Start", gamepad1.start);
            telemetry.addData("Left Stick Button", gamepad1.left_stick_button);
            telemetry.addData("Right Stick Button", gamepad1.right_stick_button);

            telemetry.addLine();

            // Gamepad 2
            telemetry.addLine("Gamepad 2:");
            telemetry.addData("Left Stick X", String.format("%.4f", gamepad2.left_stick_x));
            telemetry.addData("Left Stick Y", String.format("%.4f", gamepad2.left_stick_y));
            telemetry.addData("Right Stick X", String.format("%.4f", gamepad2.right_stick_x));
            telemetry.addData("Right Stick Y", String.format("%.4f", gamepad2.right_stick_y));
            telemetry.addData("DPad Up", gamepad2.dpad_up);
            telemetry.addData("DPad Down", gamepad2.dpad_down);
            telemetry.addData("DPad Left", gamepad2.dpad_left);
            telemetry.addData("DPad Right", gamepad2.dpad_right);
            telemetry.addData("A", gamepad2.a);
            telemetry.addData("B", gamepad2.b);
            telemetry.addData("X", gamepad2.x);
            telemetry.addData("Y", gamepad2.y);
            telemetry.addData("Left Bumper", gamepad2.left_bumper);
            telemetry.addData("Right Bumper", gamepad2.right_bumper);
            telemetry.addData("Left Trigger", String.format("%.2f", gamepad2.left_trigger));
            telemetry.addData("Right Trigger", String.format("%.2f", gamepad2.right_trigger));
            telemetry.addData("Back", gamepad2.back);
            telemetry.addData("Start", gamepad2.start);
            telemetry.addData("Left Stick Button", gamepad2.left_stick_button);
            telemetry.addData("Right Stick Button", gamepad2.right_stick_button);

            telemetry.update();
        }
    }
}
