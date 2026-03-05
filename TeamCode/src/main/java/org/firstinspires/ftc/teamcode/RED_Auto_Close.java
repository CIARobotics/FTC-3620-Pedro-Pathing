package org.firstinspires.ftc.teamcode;

// Import required libraries for FTC OpMode and Pedro Pathing
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
 * RED_Auto_Close autonomous OpMode.
 * Handles the Red side autonomous sequence starting from the "Close" position.
 * Features complex state machine logic for collecting and shooting game elements.
 */
@Autonomous(name = "RED_Auto_Close", group = "Autonomous")
@Configurable // Dashboard configuration enabled
public class RED_Auto_Close extends OpMode {
    // Manages the display of data on the Driver Station and dashboard
    private TelemetryManager panelsTelemetry;
    // Follower object for motor control and path tracking
    public Follower follower;
    // Current step in the autonomous state machine
    private int pathState;
    // Object containing all pre-built path definitions
    private Paths paths;
    // Timer for state-based delays and timed actions
    private Timer timer;

    // Motor objects for robot subsystems
    private DcMotor CL_1, CL_2, HAR_MTR;

    /**
     * Initialization logic run once when "INIT" is pressed.
     */
    @Override
    public void init() {
        // Setup telemetry and follower
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        
        // Initialize mechanism motors
        CL_1 = hardwareMap.get(DcMotor.class, "CL_1");
        CL_2 = hardwareMap.get(DcMotor.class, "CL_2");
        HAR_MTR = hardwareMap.get(DcMotor.class, "HAR_MTR");

        // Set braking behavior for precision stopping
        CL_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CL_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HAR_MTR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure motor directions
        CL_1.setDirection(DcMotorSimple.Direction.FORWARD);
        CL_2.setDirection(DcMotorSimple.Direction.FORWARD);
        HAR_MTR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set initial field location for the Red Close starting point
        follower.setStartingPose(new Pose(107.261, 135.409, Math.toRadians(90)));
        // Pre-build all the paths
        paths = new Paths(follower);
        // Start the timer
        timer = new Timer();

        // Update telemetry status
        panelsTelemetry.debug("Status", "Initialized RED_Auto_Close");
        panelsTelemetry.update(telemetry);
    }

    /**
     * Loops continuously while the OpMode is active.
     */
    @Override
    public void loop() {
        // MUST be called every loop to update localization and pathing math
        follower.update();
        // Execute state machine logic
        autonomousPathUpdate();

        // Send debugging information to the telemetry dashboard
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    /** Sets mechanisms to intake balls into the robot. */
    public void startLoading() {
        CL_1.setPower(0.5);
        CL_2.setPower(-0.5);
    }

    /** Sets mechanisms to fire balls out of the robot. */
    public void startShooting() {
        CL_1.setPower(-1.0);
        CL_2.setPower(1.0);
    }

    /** Stops all loading and shooting mechanism motors. */
    public void stopShooting() {
        CL_1.setPower(0);
        CL_2.setPower(0);
    }

    /** Sets harvester motor power for intaking. */
    public void startCollecting() {
        HAR_MTR.setPower(0.65);
    }

    /** Stops the harvester motor. */
    public void stopCollecting() {
        HAR_MTR.setPower(0);
    }

    /**
     * Inner class to define all the Bezier paths for this OpMode.
     */
    public static class Paths {
        public PathChain Path1, Path2, Path4a, Path4b, Path5, Path6, Path7a, Path7b, Path8, Path9, Path10;

        public Paths(Follower follower) {
            // Path 1: Move from start towards first intermediate point
            Path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(107.261, 135.409), new Pose(121.938, 120.018)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(30))
                .build();

            // Path 2: Move to intermediate location before collection
            Path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(121.938, 120.018), new Pose(99.758, 77.849)))
                .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                .build();

            // Movement constraints for slow collection and fast travel
            PathConstraints slowConstraints = new PathConstraints(0.03, 30, 1, 1);
            PathConstraints fastConstraints = new PathConstraints(1.0, 30, 1, 1);

            // Split Path 4 into segments 4a and 4b to allow for a timed pause mid-move
            Path4a = follower.pathBuilder(slowConstraints)
                .addPath(new BezierLine(new Pose(99.758, 77.849), new Pose(118.348, 77.639)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
            Path4b = follower.pathBuilder(slowConstraints)
                .addPath(new BezierLine(new Pose(118.348, 77.639), new Pose(136.938, 77.428)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

            // Path 5: Return to shoot position after first collection
            Path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(136.938, 77.428), new Pose(122.533, 119.809)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

            // Path 6: Move to intermediate location before second collection
            Path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(122.533, 119.809), new Pose(98.857, 53.053)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

            // Split Path 7 into segments 7a and 7b for mid-move pause
            Path7a = follower.pathBuilder(slowConstraints)
                .addPath(new BezierLine(new Pose(98.857, 53.053), new Pose(121.240, 53.023)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
            Path7b = follower.pathBuilder(slowConstraints)
                .addPath(new BezierLine(new Pose(121.240, 53.023), new Pose(143.622, 52.993)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

            // Path 8: Return to shoot position after second collection
            Path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(143.622, 52.993), new Pose(120.603, 64.874)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

            // Path 9: Final approach to shot 3 location
            Path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(120.603, 64.874), new Pose(122.250, 119.600)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(40))
                .build();

            // Path 10: Full-speed move to final destination/park
            Path10 = follower.pathBuilder(fastConstraints)
                .addPath(new BezierLine(new Pose(122.250, 119.600), new Pose(125.182, 70.186)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();
        }
    }

    /**
     * State machine management.
     * Transitions between movement states and mechanism control states.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Step 0: Begin first movement (Path 1)
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1: // Step 1: Wait for Path 1 to finish, then start loading mechanism
                if (!follower.isBusy()) {
                    startLoading();
                    timer.resetTimer(); // Timer tracks the 1.5s priming period
                    setPathState(101);
                }
                break;
            case 101: // Priming Step: Wait 1.5s for Shot 1 mechanisms to reach speed
                if (timer.getElapsedTimeSeconds() > 1.5) {
                    startShooting(); // Activate firing
                    timer.resetTimer(); // Timer tracks the 0.5s firing period
                    setPathState(102);
                }
                break;
            case 102: // Firing Step: Shot 1 for 0.5 seconds
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    stopShooting(); // Deactivate mechanics
                    timer.resetTimer();
                    setPathState(103);
                }
                break;
            case 103: // Cooling Step: Brief pause after Shot 1
                if (timer.getElapsedTimeSeconds() > 0.25) {
                    timer.resetTimer();
                    setPathState(3); // Transition to next move
                }
                break;
            case 3: // Pre-Move Pause: Wait 800ms before Path 2
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path2);
                    setPathState(4);
                }
                break;
            case 4: // Wait for Path 2 finish
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(7); // Pause before collection move begins
                }
                break;
            case 7: // Pre-Collection Pause: Wait 800ms, then begin Path 4a and harvester
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    startCollecting();
                    follower.followPath(paths.Path4a);
                    setPathState(8);
                }
                break;
            case 8: // Wait for Path 4a finish
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(802); // Pause between Path 4a and 4b
                }
                break;
            case 802: // Mid-Move Pause: Wait 1.2s before continuing to 4b
                if (timer.getElapsedTimeSeconds() > 1.2) {
                    follower.followPath(paths.Path4b);
                    setPathState(803);
                }
                break;
            case 803: // Wait for Path 4b finish, then begin wiggle sequence
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(801);
                }
                break;
            case 801: // Wiggle Sequence: Oscillate heading for 2.5s to aid collection
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
                    timer.resetTimer();
                    setPathState(9); // Transition to next move
                }
                break;
            case 9: // Pre-Move Pause: Wait 800ms before Path 5
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path5);
                    setPathState(10);
                }
                break;
            case 10: // During Path 5: Keep collector running for first 50% of travel
                if (follower.getPathCompletion() > 0.5) {
                    stopCollecting();
                }
                if (!follower.isBusy()) {
                    startLoading(); // Arrive and prime mechanisms for Shot 2
                    timer.resetTimer();
                    setPathState(501);
                }
                break;
            case 501: // Priming Step: Wait 1.5s for Shot 2
                if (timer.getElapsedTimeSeconds() > 1.5) {
                    startShooting(); // Activate firing
                    timer.resetTimer();
                    setPathState(502);
                }
                break;
            case 502: // Firing Step: Shot 2 for 0.5 seconds
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    stopShooting(); // Deactivate mechanics
                    timer.resetTimer();
                    setPathState(503);
                }
                break;
            case 503: // Cooling Step: Pause 0.25s after Shot 2
                if (timer.getElapsedTimeSeconds() > 0.25) {
                    timer.resetTimer();
                    setPathState(12);
                }
                break;
            case 12: // Pre-Move Pause: Wait 800ms before Path 6
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path6);
                    setPathState(13);
                }
                break;
            case 13: // Wait for Path 6 finish
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14: // Pre-Collection Pause: Wait 800ms, then begin Path 7a and harvester
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    startCollecting();
                    follower.followPath(paths.Path7a);
                    setPathState(15);
                }
                break;
            case 15: // Wait for Path 7a finish
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(1502); // Pause between Path 7a and 7b
                }
                break;
            case 1502: // Mid-Move Pause: Wait 1.2s before continuing to 7b
                if (timer.getElapsedTimeSeconds() > 1.2) {
                    follower.followPath(paths.Path7b);
                    setPathState(1503);
                }
                break;
            case 1503: // Wait for Path 7b finish, then begin wiggle sequence
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(1501);
                }
                break;
            case 1501: // Wiggle Sequence: Oscillate heading for 2.5s
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
                    timer.resetTimer();
                    setPathState(16); // Transition to next move
                }
                break;
            case 16: // Pre-Move Pause: Wait 800ms before Path 8
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path8);
                    setPathState(17);
                }
                break;
            case 17: // During Path 8: Keep collector running for first 50% of travel
                if (follower.getPathCompletion() > 0.5) {
                    stopCollecting();
                }
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(18);
                }
                break;
            case 18: // Pre-Move Pause: Wait 800ms before Path 9
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path9);
                    setPathState(19);
                }
                break;
            case 19: // Wait for Path 9 finish, then begin loading for Shot 3
                if (!follower.isBusy()) {
                    startLoading();
                    timer.resetTimer();
                    setPathState(901);
                }
                break;
            case 901: // Priming Step: Wait 1.5s for Shot 3
                if (timer.getElapsedTimeSeconds() > 1.5) {
                    startShooting(); // Activate firing
                    timer.resetTimer();
                    setPathState(902);
                }
                break;
            case 902: // Firing Step: Shot 3 for 0.5 seconds
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    stopShooting(); // Deactivate mechanics
                    timer.resetTimer();
                    setPathState(903);
                }
                break;
            case 903: // Cooling Step: Pause 0.25s after Shot 3
                if (timer.getElapsedTimeSeconds() > 0.25) {
                    timer.resetTimer();
                    setPathState(21);
                }
                break;
            case 21: // Pre-Move Pause: Wait 800ms before final Move (Path 10)
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path10);
                    setPathState(22);
                }
                break;
            case 22: // Final Step: Wait for robot to park
                if (!follower.isBusy()) {
                    setPathState(-1); // Stop autonomous sequence
                }
                break;
        }
    }

    /** Updates the current state machine index. */
    public void setPathState(int state) {
        pathState = state;
    }
}
