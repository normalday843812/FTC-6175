package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor intakeMotor;
    public DcMotor scoreMotor;
    public DcMotor hangMotorL;
    public DcMotor hangMotorR;
    public CRServo intakeServo1;
    public CRServo intakeServo2;
    public Servo scoreServo;
    public Servo scorePivotServo;
    private final HardwareMap hardwareMap;

    public RobotHardware(HardwareMap hwMap) {
        hardwareMap = hwMap;
        initialize();
    }

    private void initialize() {
        // Initialize motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        scoreMotor = hardwareMap.dcMotor.get("scoreMotor");
        hangMotorL = hardwareMap.dcMotor.get("hangMotorLeft");
        hangMotorR = hardwareMap.dcMotor.get("hangMotorRight");

        // Initialize servos
        intakeServo1 = hardwareMap.crservo.get("intakeServo1");
        intakeServo2 = hardwareMap.crservo.get("intakeServo2");
        scoreServo = hardwareMap.servo.get("scoreServo");
        scorePivotServo = hardwareMap.servo.get("scorePivotServo");

        // Set motor directions
        assert frontRightMotor != null;
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        assert backRightMotor != null;
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power behavior
        assert frontLeftMotor != null;
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        assert backLeftMotor != null;
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        assert intakeMotor != null;
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        assert scoreMotor != null;
        scoreMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        assert hangMotorL != null;
        hangMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        assert hangMotorR != null;
        hangMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial motor modes
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize score motor
        initScoreMotor();

        // Set all powers to zero
        setAllPowersZero();
    }

    public void initScoreMotor() {
        // Reset encoder to clear any previous position
        scoreMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait a brief moment to ensure reset is complete
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            // Ignore interruption
        }

        // Set back to RUN_USING_ENCODER mode
        scoreMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setDrivePowers(double fl, double bl, double fr, double br) {
        frontLeftMotor.setPower(fl);
        backLeftMotor.setPower(bl);
        frontRightMotor.setPower(fr);
        backRightMotor.setPower(br);
    }

    private void setAllPowersZero() {
        setDrivePowers(0, 0, 0, 0);
        intakeMotor.setPower(0);
        scoreMotor.setPower(0);
        hangMotorL.setPower(0);
        hangMotorR.setPower(0);
        intakeServo1.setPower(0);
        intakeServo2.setPower(0);
        scoreServo.setPosition(0);
        scorePivotServo.setPosition(0.5);
    }
}