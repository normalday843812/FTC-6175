package org.firstinspires.ftc.teamcode.trajectories;

import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Vector2d;
import java.util.List;

public class Trajectory {
    private final List<TrajectoryBuilder.PathSegment> segments;
    private final double totalTime;

    public Trajectory(List<TrajectoryBuilder.PathSegment> segments) {
        this.segments = segments;

        double time = 0;
        for (TrajectoryBuilder.PathSegment segment : segments) {
            time += segment.profile.getTotalTime();
        }
        this.totalTime = time;
    }

    /**
     * Get the target pose at a specific time
     */
    public Pose2d getPose(double time) {
        if (time <= 0) return getStartPose();
        if (time >= totalTime) return getEndPose();

        // Find the active segment
        double segmentStartTime = 0;
        for (TrajectoryBuilder.PathSegment segment : segments) {
            double segmentTime = segment.profile.getTotalTime();
            if (time < segmentStartTime + segmentTime) {
                // Found the active segment
                double t = segment.profile.getPosition(time - segmentStartTime)
                        / segment.path.getLength();
                Vector2d position = segment.path.getPosition(t);
                double heading = segment.path.getHeading(t);
                return new Pose2d(position, heading);
            }
            segmentStartTime += segmentTime;
        }

        return getEndPose();
    }

    public Pose2d getStartPose() {
        return new Pose2d(
                segments.get(0).path.getPosition(0),
                segments.get(0).path.getHeading(0)
        );
    }

    public Pose2d getEndPose() {
        TrajectoryBuilder.PathSegment lastSegment = segments.get(segments.size() - 1);
        return new Pose2d(
                lastSegment.path.getPosition(1),
                lastSegment.path.getHeading(1)
        );
    }

    public double getTotalTime() {
        return totalTime;
    }
}