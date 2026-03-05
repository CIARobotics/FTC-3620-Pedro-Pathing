package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

/**
 * RED_Auto_Close autonomous with updated pauses between path segments.
 */
@Autonomous(name = "RED_Auto_Close", group = "Autonomous")
@Configurable // Panels
public class RED_Auto_Close extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Timer timer;

    // Derived Hardware
    private DcMotor CL_1, CL_2, HAR_MTR;

    /**
     * This initializes the follower, hardware, and paths.
     */
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        
        // Initialize Accessory Motors
        CL_1 = hardwareMap.get(DcMotor.class, "CL_1");
        CL_2 = hardwareMap.get(DcMotor.class, "CL_2");
        HAR_MTR = hardwareMap.get(DcMotor.class, "HAR_MTR");

        // Set Zero Power Behavior (Optional but recommended for consistency)
        CL_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CL_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HAR_MTR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Shooter motors set to FORWARD, Harvester set to REVERSE
        CL_1.setDirection(DcMotorSimple.Direction.FORWARD);
        CL_2.setDirection(DcMotorSimple.Direction.FORWARD);
        HAR_MTR.setDirection(DcMotorSimple.Direction.REVERSE);

        follower.setStartingPose(new Pose(107.261, 135.409, Math.toRadians(90)));
        paths = new Paths(follower);
        timer = new Timer();

        panelsTelemetry.debug("Status", "Initialized RED_Auto_Close");
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

    // Helper Methods for derived logic
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

    public void startCollecting() {
        HAR_MTR.setPower(0.65);
       
    }

    public void stopCollecting() {
        HAR_MTR.setPower(0);
        
    }

    public static class Paths {
        public PathChain Path1, Path2, Path4a, Path4b, Path5, Path6, Path7a, Path7b, Path8, Path9, Path10;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(107.261, 135.409), new Pose(121.938, 120.018)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(30))
                .build();

            Path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(121.938, 120.018), new Pose(99.758, 77.849)))
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                .build();

            // Slower constraints for collection paths (0.03 power)
            PathConstraints slowConstraints = new PathConstraints(0.03, 30, 1, 1);

            // Full speed constraints for path 10
            PathConstraints fastConstraints = new PathConstraints(1.0, 30, 1, 1);

            // Split Path4 into 2 segments (99.758, 77.849 to 136.938, 77.428)
            Path4a = follower.pathBuilder(slowConstraints)
                .addPath(new BezierLine(new Pose(99.758, 77.849), new Pose(118.348, 77.639)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
            Path4b = follower.pathBuilder(slowConstraints)
                .addPath(new BezierLine(new Pose(118.348, 77.639), new Pose(136.938, 77.428)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

            Path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(136.938, 77.428), new Pose(122.533, 119.809)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

            Path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(122.533, 119.809), new Pose(98.857, 53.053)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

            // Split Path7 into 2 segments (98.857, 53.053 to 143.622, 52.993)
            Path7a = follower.pathBuilder(slowConstraints)
                .addPath(new BezierLine(new Pose(98.857, 53.053), new Pose(121.240, 53.023)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
            Path7b = follower.pathBuilder(slowConstraints)
                .addPath(new BezierLine(new Pose(121.240, 53.023), new Pose(143.622, 52.993)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

            Path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(143.622, 52.993), new Pose(120.603, 64.874)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

            Path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(120.603, 64.874), new Pose(122.250, 119.600)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(40))
                .build();

            Path10 = follower.pathBuilder(fastConstraints)
                .addPath(new BezierLine(new Pose(122.250, 119.600), new Pose(125.182, 70.186)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start Path1
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1: // Wait for Path1 to finish, then Load
                if (!follower.isBusy()) {
                    startLoading();
                    timer.resetTimer();
                    setPathState(101);
                }
                break;
            case 101: // Load for 1.5s
                if (timer.getElapsedTimeSeconds() > 1.5) {
                    startShooting();
                    timer.resetTimer();
                    setPathState(102);
                }
                break;
            case 102: // Shoot for 0.5s
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    stopShooting();
                    timer.resetTimer();
                    setPathState(103);
                }
                break;
            case 103: // Wait 0.25s after stop
                if (timer.getElapsedTimeSeconds() > 0.25) {
                    timer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3: // Pause for 800ms, then Path2
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path2);
                    setPathState(4);
                }
                break;
            case 4: // Wait for Path2, then Pause
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(7); // Jump directly to pause before Path4
                }
                break;
            case 7: // Pause for 800ms, then start Collecting and Path4a
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    startCollecting();
                    follower.followPath(paths.Path4a);
                    setPathState(8);
                }
                break;
            case 8: // Wait for Path4a to finish, then pause
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(802);
                }
                break;
            case 802: // Pause 1200ms then follow Path4b
                if (timer.getElapsedTimeSeconds() > 1.2) {
                    follower.followPath(paths.Path4b);
                    setPathState(803);
                }
                break;
            case 803: // Wait for Path4b to finish, then wiggle
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(801);
                }
                break;
            case 801: // Wiggle while collecting for extra 2500ms
                double elapsed4 = timer.getElapsedTimeSeconds();
                if (elapsed4 < 0.5) {
                    follower.holdPoint(new Pose(136.938, 77.428, Math.toRadians(15)), false);
                } else if (elapsed4 < 1.0) {
                    follower.holdPoint(new Pose(136.938, 77.428, Math.toRadians(-15)), false);
                } else if (elapsed4 < 1.5) {
                    follower.holdPoint(new Pose(136.938, 77.428, Math.toRadians(15)), false);
                } else if (elapsed4 < 2.0) {
                    follower.holdPoint(new Pose(136.938, 77.428, Math.toRadians(-15)), false);
                } else {
                    follower.holdPoint(new Pose(136.938, 77.428, Math.toRadians(0)), false);
                }

                if (elapsed4 > 2.5) {
                    // Harvester keeps running during pause and Path 5
                    timer.resetTimer();
                    setPathState(9);
                }
                break;
            case 9: // Pause for 800ms, then Path5
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path5);
                    setPathState(10);
                }
                break;
            case 10: // Wait for Path5 to finish, then Load
                // Stop collecting after 50% of path 5 travel
                if (follower.getPathCompletion() > 0.5) {
                    stopCollecting();
                }
                if (!follower.isBusy()) {
                    startLoading();
                    timer.resetTimer();
                    setPathState(501);
                }
                break;
            case 501: // Load for 1.5s
                if (timer.getElapsedTimeSeconds() > 1.5) {
                    startShooting();
                    timer.resetTimer();
                    setPathState(502);
                }
                break;
            case 502: // Shoot for 0.5s
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    stopShooting();
                    timer.resetTimer();
                    setPathState(503);
                }
                break;
            case 503: // Wait 0.25s after stop
                if (timer.getElapsedTimeSeconds() > 0.25) {
                    timer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12: // Pause for 800ms, then Path6
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path6);
                    setPathState(13);
                }
                break;
            case 13: // Wait for Path6 to finish, then Pause
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14: // Pause for 800ms, then start Collecting and Path7a
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    startCollecting();
                    follower.followPath(paths.Path7a);
                    setPathState(15);
                }
                break;
            case 15: // Wait for Path7a to finish, then pause
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(1502);
                }
                break;
            case 1502: // Pause 1200ms then follow Path7b
                if (timer.getElapsedTimeSeconds() > 1.2) {
                    follower.followPath(paths.Path7b);
                    setPathState(1503);
                }
                break;
            case 1503: // Wait for Path7b to finish, then wiggle
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(1501);
                }
                break;
            case 1501: // Wiggle while collecting for extra 2500ms
                double elapsed7 = timer.getElapsedTimeSeconds();
                if (elapsed7 < 0.5) {
                    follower.holdPoint(new Pose(143.622, 52.993, Math.toRadians(15)), false);
                } else if (elapsed7 < 1.0) {
                    follower.holdPoint(new Pose(143.622, 52.993, Math.toRadians(-15)), false);
                } else if (elapsed7 < 1.5) {
                    follower.holdPoint(new Pose(143.622, 52.993, Math.toRadians(15)), false);
                } else if (elapsed7 < 2.0) {
                    follower.holdPoint(new Pose(143.622, 52.993, Math.toRadians(-15)), false);
                } else {
                    follower.holdPoint(new Pose(143.622, 52.993, Math.toRadians(0)), false);
                }

                if (elapsed7 > 2.5) {
                    // Harvester keeps running during pause and Path 8
                    timer.resetTimer();
                    setPathState(16);
                }
                break;
            case 16: // Pause for 800ms, then Path8
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path8);
                    setPathState(17);
                }
                break;
            case 17: // Wait for Path8 to finish, then Pause
                // Stop collecting after 50% of path 8 travel
                if (follower.getPathCompletion() > 0.5) {
                    stopCollecting();
                }
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(18);
                }
                break;
            case 18: // Pause for 800ms, then Path9
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path9);
                    setPathState(19);
                }
                break;
            case 19: // Wait for Path9 to finish, then Load
                if (!follower.isBusy()) {
                    startLoading();
                    timer.resetTimer();
                    setPathState(901);
                }
                break;
            case 901: // Load for 1.5s
                if (timer.getElapsedTimeSeconds() > 1.5) {
                    startShooting();
                    timer.resetTimer();
                    setPathState(902);
                }
                break;
            case 902: // Shoot for 0.5s
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    stopShooting();
                    timer.resetTimer();
                    setPathState(903);
                }
                break;
            case 903: // Wait 0.25s after stop
                if (timer.getElapsedTimeSeconds() > 0.25) {
                    timer.resetTimer();
                    setPathState(21);
                }
                break;
            case 21: // Pause for 800ms, then Path10
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path10);
                    setPathState(22);
                }
                break;
            case 22: // Wait for Path10 to finish
                if (!follower.isBusy()) {
                    setPathState(-1); // Autonomous Finished
                }
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
    }
}
