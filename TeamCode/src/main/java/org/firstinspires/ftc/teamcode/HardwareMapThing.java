package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareMapThing {
    public DcMotor FLMotor = null;
    public DcMotor FRMotor = null;
    public DcMotor BLMotor = null;
    public DcMotor BRMotor = null;
    public DcMotor BucketMotor0 = null;
    public DcMotor BucketMotor1 = null;
    public Servo clawServo = null;
    public Servo clawRollServo = null;
    public Servo clawPitchServo = null;
    public Servo ArmServo0 = null;
    public Servo ArmServo1 = null;
    public Servo ArmPitchServo0 = null;
    public Servo ArmPitchServo1 = null;
    public static final double CLAW_MIN_POSITION = 0.14;
    public static final double CLAW_MAX_POSITION = 0.39;
    public static final double DEADZONE = 0.1;
    public static final double MAX_SERVO_SPEED = 1.0;

    public HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Drive motors
        FLMotor = hwMap.get(DcMotor.class, "FLMotor");
        FRMotor = hwMap.get(DcMotor.class, "FRMotor");
        BLMotor = hwMap.get(DcMotor.class, "BLMotor");
        BRMotor = hwMap.get(DcMotor.class, "BRMotor");

        // Bucket Motors
        BucketMotor0 = hwMap.get(DcMotor.class, "BucketMotor0");
        BucketMotor1 = hwMap.get(DcMotor.class, "BucketMotor1");

        // Motor directions
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);

        // Motor modes
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BucketMotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BucketMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BucketMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BucketMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo Init
        // Claw
        clawServo = hwMap.get(Servo.class, "clawServo");
        clawRollServo = hwMap.get(Servo.class, "clawRollServo");
        clawPitchServo = hwMap.get(Servo.class, "clawPitchServo");
        // Arm
        ArmServo0 = hwMap.get(Servo.class, "ArmServo0");
        ArmServo1 = hwMap.get(Servo.class, "ArmServo1");
        ArmPitchServo0 = hwMap.get(Servo.class, "ArmPitchServo0");
        ArmPitchServo1 = hwMap.get(Servo.class, "ArmPitchServo1");

        // Init positions
        clawServo.setPosition(CLAW_MIN_POSITION);
        clawRollServo.setPosition(0.5);
        clawPitchServo.setPosition(0.5);

        // Init powers
        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);

        BucketMotor0.setPower(0);
        BucketMotor1.setPower(0);
    }
}