package org.firstinspires.ftc.teamcode.trajectories;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Builder class for creating multi-segment trajectories.
 * Coordinate system:
 * - Positive X is forward
 * - Positive Y is right
 * - Positive heading is clockwise
 */
@Config
public class TrajectoryBuilder {
    public static class Params {
        public static double maxVel = 1000.0;    // mm/s
        public static double maxAccel = 500.0;   // mm/s²
        public static double maxAngVel = Math.PI; // rad/s
        public static double maxAngAccel = Math.PI/2; // rad/s²
    }

    private final List<PathSegment> segments;
    private Pose2d lastPose;

    public static class PathSegment {
        public final SplinePath path;
        public final MotionProfile profile;

        public PathSegment(SplinePath path, MotionProfile profile) {
            this.path = path;
            this.profile = profile;
        }
    }

    public TrajectoryBuilder(Pose2d startPose) {
        this.segments = new ArrayList<>();
        this.lastPose = startPose;
    }

    /**
     * Add a straight line movement to the trajectory
     */
    public TrajectoryBuilder lineTo(Vector2d endPosition) {
        Pose2d endPose = new Pose2d(endPosition, lastPose.heading);
        SplinePath path = new SplinePath(lastPose, endPose);
        MotionProfile profile = new MotionProfile(0, path.getLength(),
                Params.maxVel, Params.maxAccel);

        segments.add(new PathSegment(path, profile));
        lastPose = endPose;
        return this;
    }

    /**
     * Add a line segment with specific heading
     */
    public TrajectoryBuilder lineToLinearHeading(Pose2d endPose) {
        SplinePath path = new SplinePath(lastPose, endPose);
        MotionProfile profile = new MotionProfile(0, path.getLength(),
                Params.maxVel, Params.maxAccel);

        segments.add(new PathSegment(path, profile));
        lastPose = endPose;
        return this;
    }

    /**
     * Add a spline segment to the trajectory
     */
    public TrajectoryBuilder splineTo(Pose2d endPose) {
        SplinePath path = new SplinePath(lastPose, endPose);
        MotionProfile profile = new MotionProfile(0, path.getLength(),
                Params.maxVel, Params.maxAccel);

        segments.add(new PathSegment(path, profile));
        lastPose = endPose;
        return this;
    }

    /**
     * Add a turn in place to the trajectory
     */
    public TrajectoryBuilder turn(double angle) {  // Positive = clockwise
        double startHeading = lastPose.heading;
        double endHeading = startHeading + angle;

        MotionProfile profile = new MotionProfile(
                startHeading, endHeading,
                Params.maxAngVel, Params.maxAngAccel
        );

        lastPose = new Pose2d(lastPose.position, endHeading);
        segments.add(new PathSegment(
                new SplinePath(lastPose, lastPose),  // Zero-distance path
                profile
        ));

        return this;
    }

    /**
     * Add a turn to face a specific point
     */
    public TrajectoryBuilder turnTo(Vector2d point) {
        Vector2d relative = point.minus(lastPose.position);
        double targetHeading = relative.angle();
        double angle = targetHeading - lastPose.heading;

        return turn(angle);
    }

    /**
     * Add a forward movement
     */
    public TrajectoryBuilder forward(double distance) {  // Positive = forward
        Vector2d endPosition = lastPose.position.plus(
                new Vector2d(
                        distance * Math.cos(lastPose.heading),
                        distance * Math.sin(lastPose.heading)
                )
        );
        return lineTo(endPosition);
    }

    /**
     * Add a backward movement
     */
    public TrajectoryBuilder back(double distance) {
        return forward(-distance);
    }

    /**
     * Add a strafe movement
     */
    public TrajectoryBuilder strafeLeft(double distance) {  // Positive = left = -Y
        Vector2d endPosition = lastPose.position.plus(
                new Vector2d(
                        distance * Math.cos(lastPose.heading - Math.PI/2),
                        distance * Math.sin(lastPose.heading - Math.PI/2)
                )
        );
        return lineTo(endPosition);
    }

    /**
     * Add a right strafe movement
     */
    public TrajectoryBuilder strafeRight(double distance) {
        return strafeLeft(-distance);
    }

    /**
     * Build the complete trajectory
     */
    public Trajectory build() {
        return new Trajectory(segments);
    }

    public Pose2d getCurrentPose() {
        return lastPose;
    }
}