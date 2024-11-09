package org.firstinspires.ftc.teamcode.roadrunnerpkg.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MecanumLocalizerInputsMessage {
    public long timestamp;
    public PositionVelocityPair leftFrontPosVel;
    public PositionVelocityPair leftBackPosVel;
    public PositionVelocityPair rightBackPosVel;
    public PositionVelocityPair rightFrontPosVel;
    public YawPitchRollAngles angles;

    public MecanumLocalizerInputsMessage(
            PositionVelocityPair leftFrontPosVel,
            PositionVelocityPair leftBackPosVel,
            PositionVelocityPair rightBackPosVel,
            PositionVelocityPair rightFrontPosVel,
            YawPitchRollAngles angles
    ) {
        this.timestamp = System.nanoTime();
        this.leftFrontPosVel = leftFrontPosVel;
        this.leftBackPosVel = leftBackPosVel;
        this.rightBackPosVel = rightBackPosVel;
        this.rightFrontPosVel = rightFrontPosVel;
        this.angles = angles;
    }
}