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
        // TODO: tune these values friday/comp, they're probably wrong
        public static double maxVel = 1000.0;    // mm/s
        public static double maxAccel = 500.0;   // mm/s²
        public static double maxAngVel = Math.PI; // rad/s
        public static double maxAngAccel = Math.PI/2; // rad/s²
    }

    private final List<PathSegment> segments;
    private Pose2d lastPose;

    // Container for path segments with their motion profiles
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
        MotionProfile profile = new MotionProfile(0, path.getLength(), Params.maxVel, Params.maxAccel);

        segments.add(new PathSegment(path, profile));
        lastPose = endPose;
        return this;
    }

    /**
     * Add a line segment with specific heading
     */
    public TrajectoryBuilder lineToLinearHeading(Pose2d endPose) {
        SplinePath path = new SplinePath(lastPose, endPose);
        MotionProfile profile = new MotionProfile(0, path.getLength(), Params.maxVel, Params.maxAccel);

        segments.add(new PathSegment(path, profile));
        lastPose = endPose;
        return this;
    }

    /**
     * Add a spline segment to the trajectory
     * TODO: no touchies or robot have a seizure
     */
    public TrajectoryBuilder splineTo(Pose2d endPose) {
        SplinePath path = new SplinePath(lastPose, endPose);
        MotionProfile profile = new MotionProfile(0, path.getLength(), Params.maxVel, Params.maxAccel);

        segments.add(new PathSegment(path, profile));
        lastPose = endPose;
        return this;
    }

    /**
     * Add a turn in place to the trajectory
     */
    // TODO: if the robot spins in circles check the sign of this angle
    public TrajectoryBuilder turn(double angle) {
        double startHeading = lastPose.heading;
        double endHeading = startHeading + angle;  // Positive angle = clockwise

        MotionProfile profile = new MotionProfile(
                startHeading, endHeading,
                Params.maxAngVel, Params.maxAngAccel
        );

        // Create a zero-distance path that just changes heading
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
    public TrajectoryBuilder forward(double distance) {
        Vector2d endPosition = lastPose.position.plus(
                new Vector2d(
                        distance * Math.cos(lastPose.heading),  // Forward = +X
                        distance * Math.sin(lastPose.heading)
                )
        );
        return lineTo(endPosition);
    }

    /**
     * Add a backward movement
     */
    public TrajectoryBuilder back(double distance) {
        return forward(-distance);  // Backward = -X
    }

    /**
     * Add a strafe movement
     * TODO: The old code had +PI/2 here which was wrong, if something breaks check this first
     */
    public TrajectoryBuilder strafeLeft(double distance) {
        Vector2d endPosition = lastPose.position.plus(
                new Vector2d(
                        distance * Math.cos(lastPose.heading - Math.PI/2),  // Left = -Y = -PI/2
                        distance * Math.sin(lastPose.heading - Math.PI/2)
                )
        );
        return lineTo(endPosition);
    }

    /**
     * Add a right strafe movement
     */
    public TrajectoryBuilder strafeRight(double distance) {
        return strafeLeft(-distance);  // Right = +Y
    }

    /**
     * Build the complete trajectory
     */
    // I spent 2 hours debugging why the robot wouldn't move only to realize
    // I forgot to call build() fuck me
    public Trajectory build() {
        return new Trajectory(segments);
    }

    /**
     * Get the current end pose of the trajectory
     */
    public Pose2d getCurrentPose() {
        return lastPose;
    }
}