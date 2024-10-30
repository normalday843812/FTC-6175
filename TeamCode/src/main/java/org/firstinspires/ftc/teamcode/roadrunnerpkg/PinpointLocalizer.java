package org.firstinspires.ftc.teamcode.roadrunnerpkg;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointLocalizer implements Localizer {
    private final GoBildaPinpointDriver pinpoint;
    private boolean initialized = false;
    private double lastHeading = 0;
    private Pose2D lastPose;

    public PinpointLocalizer(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;

        // Configure Pinpoint
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-84.0, -168.0); // Adjust these values for your robot setup
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
    }

    @Override
    public Twist2dDual<Time> update() {
        pinpoint.update();

        Pose2D currentPose = pinpoint.getPosition();

        if (!initialized) {
            initialized = true;
            lastPose = currentPose;
            lastHeading = currentPose.getHeading(AngleUnit.RADIANS);
            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        // Convert mm to inches for RoadRunner compatibility
        double MM_PER_INCH = 25.4;

        double dx = (currentPose.getX(DistanceUnit.MM) - lastPose.getX(DistanceUnit.MM)) / MM_PER_INCH;
        double dy = (currentPose.getY(DistanceUnit.MM) - lastPose.getY(DistanceUnit.MM)) / MM_PER_INCH;
        double dh = currentPose.getHeading(AngleUnit.RADIANS) - lastHeading;

        Pose2D velocity = pinpoint.getVelocity();
        double vx = velocity.getX(DistanceUnit.MM) / MM_PER_INCH;
        double vy = velocity.getY(DistanceUnit.MM) / MM_PER_INCH;
        double vh = velocity.getHeading(AngleUnit.RADIANS);

        lastPose = currentPose;
        lastHeading = currentPose.getHeading(AngleUnit.RADIANS);

        return new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[] {dx, vx}),
                        new DualNum<>(new double[] {dy, vy})
                ),
                new DualNum<>(new double[] {dh, vh})
        );
    }
}