package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Controller Test", group = "Test")
public class ControllerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        boolean useGamepad1 = true;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
                useGamepad1 = !useGamepad1;
                sleep(300);
            }

            if (useGamepad1) {
                telemetry.addData("Active Gamepad", "Gamepad 1");
                // Buttons
                telemetry.addData("Button A", gamepad1.a);
                telemetry.addData("Button B", gamepad1.b);
                telemetry.addData("Button X", gamepad1.x);
                telemetry.addData("Button Y", gamepad1.y);
                telemetry.addData("Button Start", gamepad1.start);
                telemetry.addData("Button Back", gamepad1.back);
                telemetry.addData("Left Bumper", gamepad1.left_bumper);
                telemetry.addData("Right Bumper", gamepad1.right_bumper);
                telemetry.addData("Left Stick Button", gamepad1.left_stick_button);
                telemetry.addData("Right Stick Button", gamepad1.right_stick_button);

                // D-Pad
                telemetry.addData("DPad Up", gamepad1.dpad_up);
                telemetry.addData("DPad Down", gamepad1.dpad_down);
                telemetry.addData("DPad Left", gamepad1.dpad_left);
                telemetry.addData("DPad Right", gamepad1.dpad_right);

                // Joysticks
                telemetry.addData("Left Stick X", gamepad1.left_stick_x);
                telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
                telemetry.addData("Right Stick X", gamepad1.right_stick_x);
                telemetry.addData("Right Stick Y", gamepad1.right_stick_y);

                // Triggers
                telemetry.addData("Left Trigger", gamepad1.left_trigger);
                telemetry.addData("Right Trigger", gamepad1.right_trigger);

                // Misc
                telemetry.addData("Guide Button", gamepad1.guide);
                telemetry.addData("Timestamp", gamepad1.timestamp);
                telemetry.addData("Options", gamepad1.options);
            } else {
                telemetry.addData("Active Gamepad", "Gamepad 2");
                // Buttons
                telemetry.addData("Button A", gamepad2.a);
                telemetry.addData("Button B", gamepad2.b);
                telemetry.addData("Button X", gamepad2.x);
                telemetry.addData("Button Y", gamepad2.y);
                telemetry.addData("Button Start", gamepad2.start);
                telemetry.addData("Button Back", gamepad2.back);
                telemetry.addData("Left Bumper", gamepad2.left_bumper);
                telemetry.addData("Right Bumper", gamepad2.right_bumper);
                telemetry.addData("Left Stick Button", gamepad2.left_stick_button);
                telemetry.addData("Right Stick Button", gamepad2.right_stick_button);

                // D-Pad
                telemetry.addData("DPad Up", gamepad2.dpad_up);
                telemetry.addData("DPad Down", gamepad2.dpad_down);
                telemetry.addData("DPad Left", gamepad2.dpad_left);
                telemetry.addData("DPad Right", gamepad2.dpad_right);

                // Joysticks
                telemetry.addData("Left Stick X", gamepad2.left_stick_x);
                telemetry.addData("Left Stick Y", gamepad2.left_stick_y);
                telemetry.addData("Right Stick X", gamepad2.right_stick_x);
                telemetry.addData("Right Stick Y", gamepad2.right_stick_y);

                // Triggers
                telemetry.addData("Left Trigger", gamepad2.left_trigger);
                telemetry.addData("Right Trigger", gamepad2.right_trigger);

                // Misc
                telemetry.addData("Guide Button", gamepad2.guide);
                telemetry.addData("Timestamp", gamepad2.timestamp);
                telemetry.addData("Options", gamepad2.options);
            }

            // Update telemetry
            telemetry.update();
        }
    }
}
