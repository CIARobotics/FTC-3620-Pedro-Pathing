package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

/**
 * RED_Auto_LONG autonomous.
 * Starts with a 20-second delay, then follows a path to shoot with pauses between points.
 * Mirrored from BLUE_Auto_LONG.
 */
@Autonomous(name = "RED_Auto_LONG", group = "Autonomous")
@Configurable // Panels
public class RED_Auto_LONG extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Timer timer;

    // Hardware
    private DcMotor CL_1, CL_2, HAR_MTR;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        
        // Initialize Accessory Motors
        CL_1 = hardwareMap.get(DcMotor.class, "CL_1");
        CL_2 = hardwareMap.get(DcMotor.class, "CL_2");
        HAR_MTR = hardwareMap.get(DcMotor.class, "HAR_MTR");

        CL_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CL_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HAR_MTR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter motors set to FORWARD, Harvester set to REVERSE
        CL_1.setDirection(DcMotorSimple.Direction.FORWARD);
        CL_2.setDirection(DcMotorSimple.Direction.FORWARD);
        HAR_MTR.setDirection(DcMotorSimple.Direction.REVERSE);

        follower.setStartingPose(new Pose(88.000, 8.000, Math.toRadians(90)));
        paths = new Paths(follower);
        timer = new Timer();

        panelsTelemetry.debug("Status", "Initialized RED_Auto_LONG");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    // Helper Methods
    public void startLoading() {
        CL_1.setPower(0.5);
        CL_2.setPower(-0.5);
    }

    public void startShooting() {
        CL_1.setPower(-1.0);
        CL_2.setPower(1.0);
    }

    public void stopShooting() {
        CL_1.setPower(0);
        CL_2.setPower(0);
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3;

        public Paths(Follower follower) {
            PathConstraints fastConstraints = new PathConstraints(1.0, 30, 1, 1);

            Path1 = follower.pathBuilder(fastConstraints)
                .addPath(new BezierLine(new Pose(88.000, 8.000), new Pose(84.809, 250.990)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(40))
                .build();

            Path2 = follower.pathBuilder(fastConstraints)
                .addPath(new BezierLine(new Pose(84.809, 250.990), new Pose(117.068, 119.625)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(40))
                .build();

            Path3 = follower.pathBuilder(fastConstraints)
                .addPath(new BezierLine(new Pose(117.068, 119.625), new Pose(128.824, 83.235)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start: Idle for 20 seconds
                if (timer.getElapsedTimeSeconds() > 20.0) {
                    follower.followPath(paths.Path1);
                    setPathState(1);
                }
                break;
            case 1: // Wait for Path 1
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(7); // Pause after Path 1
                }
                break;
            case 7: // Pause for 800ms
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;
            case 2: // Wait for Path 2
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(8); // Pause after Path 2
                }
                break;
            case 8: // Pause for 800ms before loading
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    startLoading();
                    timer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3: // Load for 1.5s
                if (timer.getElapsedTimeSeconds() > 1.5) {
                    startShooting();
                    timer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4: // Shoot for 0.5s
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    stopShooting();
                    timer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5: // Wait 800ms after stop
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path3);
                    setPathState(6);
                }
                break;
            case 6: // Wait for Path 3 finish
                if (!follower.isBusy()) {
                    setPathState(-1); // Finished
                }
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
    }
}
