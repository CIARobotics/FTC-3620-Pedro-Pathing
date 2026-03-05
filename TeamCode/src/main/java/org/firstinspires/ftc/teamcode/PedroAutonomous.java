package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

/**
 * This is the PedroAutonomous OpMode. It runs the robot in a sequence of paths
 * defined using the Pedro Pathing visualizer.
 */
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    /**
     * This initializes the follower, the starting pose, and the paths.
     */
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        
        // Note: Ensure your starting pose matches your first path's start point or the robot will move there immediately.
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    /**
     * This is the main loop of the OpMode. It updates the follower and the state machine.
     */
    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    /**
     * This class contains all the paths we want the robot to follow.
     */
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(107.261, 135.409),
                        new Pose(115.770, 124.697)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();

            Path2 = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(115.770, 124.697),
                        new Pose(100.183, 84.018)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(65))
                .build();

            Path3 = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(100.183, 84.018),
                        new Pose(102.260, 83.979)
                    )
                )
                .setTangentHeadingInterpolation()
                .build();

            Path4 = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(102.260, 83.979),
                        new Pose(128.004, 83.597)
                    )
                )
                .setTangentHeadingInterpolation()
                .build();

            Path5 = follower.pathBuilder()
                .addPath(
                    new BezierLine(
                        new Pose(128.004, 83.597),
                        new Pose(118.917, 123.851)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
        }
    }

    /**
     * This is the state machine that manages the autonomous sequence.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start following Path1
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1: // Wait for Path1 to finish
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;
            case 2: // Wait for Path2 to finish
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    setPathState(3);
                }
                break;
            case 3: // Wait for Path3 to finish
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    setPathState(4);
                }
                break;
            case 4: // Wait for Path4 to finish
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    setPathState(5);
                }
                break;
            case 5: // All paths finished
                if (!follower.isBusy()) {
                    // Autonomous Finished
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * Helper method to change the path state.
     */
    public void setPathState(int state) {
        pathState = state;
    }
}
