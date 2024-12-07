package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "RTKServoTest", group = "TeleOp")
public class RTKServoTest extends LinearOpMode {
    private double servoPosition = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo ArmServo0 = hardwareMap.get(Servo.class, "ArmServo0");
        Servo ArmServo1 = hardwareMap.get(Servo.class, "ArmServo1");

        ArmServo0.setPosition(servoPosition);
        ArmServo1.setPosition(1.0 - servoPosition);

        telemetry.addData("Status", "Initialized");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double SERVO_INCREMENT = 0.00175;
            if (gamepad1.a) {
                servoPosition += SERVO_INCREMENT;
            }
            else if (gamepad1.b) {
                servoPosition -= SERVO_INCREMENT;
            }

            servoPosition = Math.max(0.0, Math.min(1.0, servoPosition));

            ArmServo0.setPosition(servoPosition);
            ArmServo1.setPosition(1.0 - servoPosition);
            telemetry.update();
        }
    }
}
