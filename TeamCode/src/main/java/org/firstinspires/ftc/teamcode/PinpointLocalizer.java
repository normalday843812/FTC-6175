package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;

public class PinpointLocalizer implements Localizer {
    private final GoBildaPinpointDriver pinpoint;
    private static final double MM_PER_INCH = 25.4;

    private double lastXMM = 0;
    private double lastYMM = 0;
    private double lastHeading = 0;
    private boolean initialized = false;

    public PinpointLocalizer(GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;
    }

    @Override
    public Twist2dDual<Time> update() {
        pinpoint.update();

        // Get current positions and velocities
        double currentXMM = pinpoint.getPosX();
        double currentYMM = pinpoint.getPosY();
        double currentHeading = pinpoint.getHeading();

        if (!initialized) {
            initialized = true;
            lastXMM = currentXMM;
            lastYMM = currentYMM;
            lastHeading = currentHeading;

            // Create Vector2dDual with zero displacement and velocity
            Vector2dDual<Time> line = new Vector2dDual<>(
                    DualNum.constant(0.0, 2),  // x component
                    DualNum.constant(0.0, 2)   // y component
            );

            // Create DualNum with zero rotation and velocity
            DualNum<Time> angle = DualNum.constant(0.0, 2);

            return new Twist2dDual<>(line, angle);
        }

        // Calculate position changes (in millimeters)
        double dxMM = currentXMM - lastXMM;
        double dyMM = currentYMM - lastYMM;
        double dh = currentHeading - lastHeading;

        // Get velocities
        double xVelMM = pinpoint.getVelX();
        double yVelMM = pinpoint.getVelY();
        double headingVel = pinpoint.getHeadingVelocity();

        // Update stored positions
        lastXMM = currentXMM;
        lastYMM = currentYMM;
        lastHeading = currentHeading;

        // Create the Vector2dDual for linear motion
        Vector2dDual<Time> line = new Vector2dDual<>(
                // x component: displacement and velocity
                new DualNum<>(new double[] {
                        dxMM / MM_PER_INCH,    // Convert displacement to inches
                        xVelMM / MM_PER_INCH   // Convert velocity to inches/sec
                }),
                // y component: displacement and velocity
                new DualNum<>(new double[] {
                        dyMM / MM_PER_INCH,    // Convert displacement to inches
                        yVelMM / MM_PER_INCH   // Convert velocity to inches/sec
                })
        );

        // Create the DualNum for angular motion
        DualNum<Time> angle = new DualNum<>(new double[] {
                dh,          // Change in heading (radians)
                headingVel   // Angular velocity (radians/sec)
        });

        // Construct and return the Twist2dDual
        return new Twist2dDual<>(line, angle);
    }
}