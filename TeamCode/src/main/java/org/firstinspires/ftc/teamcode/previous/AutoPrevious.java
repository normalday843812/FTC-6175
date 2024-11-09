package org.firstinspires.ftc.teamcode.previous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AutoPrevious extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        // Initialize motors
        frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        backRight = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse right side motors
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Add telemetry to confirm initialization
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        driveLeft();
        sleep(500);

        stopDriving();


//        // Drive forward for 1 second
//        telemetry.addData("Status", "Driving Forward");
//        telemetry.update();
//        driveForward();
//        sleep(1000);
//
//        // Stop briefly
//        stopDriving();
//        sleep(50);
//
//        // Strafe left for 1 second
//        telemetry.addData("Status", "Strafing Left");
//        telemetry.update();
//        driveLeft();
//        sleep(1000);
//
//        // Stop
//        stopDriving();
//        telemetry.addData("Status", "Complete");
//        telemetry.update();
    }

    private void driveForward() {
        frontLeft.setPower(1.0);
        backLeft.setPower(1.0);
        frontRight.setPower(1.0);
        backRight.setPower(1.0);
    }
    private void driveBackward() {
        frontLeft.setPower(-1.0);
        backLeft.setPower(-1.0);
        frontRight.setPower(-1.0);
        backRight.setPower(-1.0);
    }

    private void stopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void driveLeft() {
        // Correct power values for mecanum wheel strafing
        frontLeft.setPower(-1.0);
        backLeft.setPower(1.0);
        frontRight.setPower(1.0);
        backRight.setPower(-1.0);
    }
    private void driveRight() {
        // Correct power values for mecanum wheel strafing
        frontLeft.setPower(1.0);
        backLeft.setPower(-1.0);
        frontRight.setPower(-1.0);
        backRight.setPower(1.0);
    }
}