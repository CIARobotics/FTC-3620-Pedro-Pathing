package org.firstinspires.ftc.teamcode;

// Import core FTC SDK components for OpMode and hardware control
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// Import dashboard and Pedro Pathing specific utilities
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
 * BLUE_Auto_Close autonomous OpMode.
 * This class coordinates the Blue side autonomous sequence starting from the "Close" position.
 * It uses a state machine to manage path following, mechanism pauses, and harvesting/shooting logic.
 */
@Autonomous(name = "BLUE_Auto_Close", group = "Autonomous")
@Configurable // Integrated with Panels dashboard for real-time adjustments
public class BLUE_Auto_Close extends OpMode {
    // Manages custom telemetry data formatted for the dashboard
    private TelemetryManager panelsTelemetry;
    // The Follower object manages the robot's pose and moves the motors along paths
    public Follower follower;
    // An integer representing our current step in the switch-case state machine
    private int pathState;
    // Holder for all the path trajectories defined in the inner class below
    private Paths paths;
    // Utility for tracking time elapsed during specific states
    private Timer timer;

    // Mechanism motor declarations
    private DcMotor CL_1, CL_2, HAR_MTR;

    /**
     * The init method is executed once after the user presses the "INIT" button.
     */
    @Override
    public void init() {
        // Initialize the custom telemetry manager
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        // Create the Pedro Pathing follower using pre-defined project constants
        follower = Constants.createFollower(hardwareMap);
        
        // Map the hardware motor variables to the configuration names on the Control Hub
        CL_1 = hardwareMap.get(DcMotor.class, "CL_1");
        CL_2 = hardwareMap.get(DcMotor.class, "CL_2");
        HAR_MTR = hardwareMap.get(DcMotor.class, "HAR_MTR");

        // Set motors to BRAKE mode to prevent drifting after power is cut
        CL_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CL_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HAR_MTR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure the rotation direction for each mechanism motor
        CL_1.setDirection(DcMotorSimple.Direction.FORWARD);
        CL_2.setDirection(DcMotorSimple.Direction.FORWARD);
        HAR_MTR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Tell the robot exactly where it is starting on the field map
        follower.setStartingPose(new Pose(37.282, 135.622, Math.toRadians(90)));
        // Generate all the Bezier trajectories
        paths = new Paths(follower);
        // Initialize the timer for our state machine
        timer = new Timer();

        // Log initialization complete status
        panelsTelemetry.debug("Status", "Initialized BLUE_Auto_Close");
        panelsTelemetry.update(telemetry);
    }

    /**
     * The loop method runs continuously while the OpMode is active.
     */
    @Override
    public void loop() {
        // MUST call follower.update() every loop to process odometry and motor power
        follower.update();
        // Check the state machine for the next action to perform
        autonomousPathUpdate();

        // Continuously send diagnostics to the telemetry dashboard
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    /** Helper method to power mechanisms for loading elements into the shooter. */
    public void startLoading() {
        CL_1.setPower(0.5);
        CL_2.setPower(-0.5);
    }

    /** Helper method to power mechanisms for firing. */
    public void startShooting() {
        CL_1.setPower(-1.0);
        CL_2.setPower(1.0);
    }

    /** Helper method to turn off all mechanism motors. */
    public void stopShooting() {
        CL_1.setPower(0);
        CL_2.setPower(0);
    }

    /** Helper method to start the harvester motor for intake. */
    public void startCollecting() {
        HAR_MTR.setPower(0.65);
    }

    /** Helper method to stop the harvester motor. */
    public void stopCollecting() {
        HAR_MTR.setPower(0);
    }

    /**
     * Static inner class containing all the pre-built paths for this autonomous OpMode.
     */
    public static class Paths {
        // Definition of all path chains used in the sequence
        public PathChain Path1, Path2, Path4a, Path4b, Path5, Path6, Path7a, Path7b, Path8, Path9, Path10;

        public Paths(Follower follower) {
            // Path 1: Initial drive segment
            Path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(37.282, 135.622), new Pose(26.009, 118.316)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
                .build();

            // Path 2: Secondary move segment
            Path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(26.009, 118.316), new Pose(42.966, 74.233)))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .build();

            // Custom constraints for collection (slow) and travel (fast)
            PathConstraints slowConstraints = new PathConstraints(0.03, 30, 1, 1);
            PathConstraints fastConstraints = new PathConstraints(1.0, 30, 1, 1);

            // Path 4 Split: Moves toward the first collection point with a timed pause in the middle
            Path4a = follower.pathBuilder(slowConstraints)
                .addPath(new BezierLine(new Pose(42.966, 74.233), new Pose(27.949, 74.129)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
            Path4b = follower.pathBuilder(slowConstraints)
                .addPath(new BezierLine(new Pose(27.949, 74.129), new Pose(12.932, 74.025)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

            // Path 5: Return to shoot after first harvest
            Path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(12.932, 74.025), new Pose(25.966, 118.108)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

            // Path 6: Move to intermediate position before second harvest
            Path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(25.966, 118.108), new Pose(45.043, 48.799)))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .build();

            // Path 7 Split: Moves toward second harvest point with a timed pause
            Path7a = follower.pathBuilder(slowConstraints)
                .addPath(new BezierLine(new Pose(45.043, 48.799), new Pose(23.077, 48.875)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
            Path7b = follower.pathBuilder(slowConstraints)
                .addPath(new BezierLine(new Pose(23.077, 48.875), new Pose(1.111, 48.951)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

            // Path 8: Return segment after second harvest
            Path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(1.111, 48.951), new Pose(23.823, 63.811)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                .build();

            // Path 9: Alignment for the final shot
            Path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(23.823, 63.811), new Pose(25.470, 118.111)))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(140))
                .build();

            // Path 10: Final high-speed move to park or end location
            Path10 = follower.pathBuilder(fastConstraints)
                .addPath(new BezierLine(new Pose(25.470, 118.111), new Pose(23.084, 67.846)))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .build();
        }
    }

    /**
     * Main autonomous state machine.
     * Controls the sequence of movements and mechanism activations.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Step 0: Start first movement segment
                follower.followPath(paths.Path1);
                setPathState(1);
                break;
            case 1: // Step 1: Wait for Path 1 to reach destination, then begin loading mechanisms
                if (!follower.isBusy()) {
                    startLoading();
                    timer.resetTimer();
                    setPathState(101);
                }
                break;
            case 101: // Priming Step: Wait 1.5s for motors to speed up
                if (timer.getElapsedTimeSeconds() > 1.5) {
                    startShooting(); // Activate the firing motor
                    timer.resetTimer();
                    setPathState(102);
                }
                break;
            case 102: // Firing Step: Firing for 0.5 seconds
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    stopShooting(); // Deactivate shooting mechanisms
                    timer.resetTimer();
                    setPathState(103);
                }
                break;
            case 103: // Post-Shot Pause: Brief cooling/stabilization delay
                if (timer.getElapsedTimeSeconds() > 0.25) {
                    timer.resetTimer();
                    setPathState(3); // Move to state for next drive segment
                }
                break;
            case 3: // Pre-Move Pause: Wait 800ms before starting Path 2
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path2);
                    setPathState(4);
                }
                break;
            case 4: // Wait for Path 2 to complete
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(7); // Pause before starting harvest move
                }
                break;
            case 7: // Pre-Harvest Pause: Wait 800ms, then start harvesting motors and move Path 4a
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    startCollecting();
                    follower.followPath(paths.Path4a);
                    setPathState(8);
                }
                break;
            case 8: // Wait for Path 4a segment to finish
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(802); // Pause mid-move
                }
                break;
            case 802: // Mid-Move Harvest Pause: Wait 1.2s before segment 4b
                if (timer.getElapsedTimeSeconds() > 1.2) {
                    follower.followPath(paths.Path4b);
                    setPathState(803);
                }
                break;
            case 803: // Wait for segment 4b finish, then begin wiggle sequence
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(801);
                }
                break;
            case 801: // Wiggle Logic: Oscillate heading back and forth for 2.5s at the harvest point
                double elapsed4 = timer.getElapsedTimeSeconds();
                if (elapsed4 < 0.5) {
                    follower.holdPoint(new Pose(12.932, 74.025, Math.toRadians(180 + 15)), false);
                } else if (elapsed4 < 1.0) {
                    follower.holdPoint(new Pose(12.932, 74.025, Math.toRadians(180 - 15)), false);
                } else if (elapsed4 < 1.5) {
                    follower.holdPoint(new Pose(12.932, 74.025, Math.toRadians(180 + 15)), false);
                } else if (elapsed4 < 2.0) {
                    follower.holdPoint(new Pose(12.932, 74.025, Math.toRadians(180 - 15)), false);
                } else {
                    follower.holdPoint(new Pose(12.932, 74.025, Math.toRadians(180)), false);
                }

                if (elapsed4 > 2.5) {
                    timer.resetTimer();
                    setPathState(9); // Harvest motors stay ON during this transition
                }
                break;
            case 9: // Pre-Move Pause: Wait 800ms before Path 5
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path5);
                    setPathState(10);
                }
                break;
            case 10: // Wait for Path 5 travel: stop harvester after 50% of the distance is traveled
                if (follower.getPathCompletion() > 0.5) {
                    stopCollecting();
                }
                if (!follower.isBusy()) {
                    startLoading(); // Prime for Shot 2
                    timer.resetTimer();
                    setPathState(501);
                }
                break;
            case 501: // Loading Shot 2: Wait 1.5 seconds
                if (timer.getElapsedTimeSeconds() > 1.5) {
                    startShooting(); // Firing Shot 2
                    timer.resetTimer();
                    setPathState(502);
                }
                break;
            case 502: // Firing Step: Firing Shot 2 for 0.5 seconds
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    stopShooting();
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
            case 13: // Wait for Path 6 to complete
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(14);
                }
                break;
            case 14: // Pre-Harvest Pause: Wait 800ms, start collecting and Path 7a
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    startCollecting();
                    follower.followPath(paths.Path7a);
                    setPathState(15);
                }
                break;
            case 15: // Wait for Path 7a segment to finish
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(1502);
                }
                break;
            case 1502: // Mid-Move Harvest Pause: Wait 1.2s before segment 7b
                if (timer.getElapsedTimeSeconds() > 1.2) {
                    follower.followPath(paths.Path7b);
                    setPathState(1503);
                }
                break;
            case 1503: // Wait for segment 7b finish, then begin wiggle
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(1501);
                }
                break;
            case 1501: // Wiggle sequence at the second harvest point for 2.5s
                double elapsed7 = timer.getElapsedTimeSeconds();
                if (elapsed7 < 0.5) {
                    follower.holdPoint(new Pose(1.111, 48.951, Math.toRadians(180 + 15)), false);
                } else if (elapsed7 < 1.0) {
                    follower.holdPoint(new Pose(1.111, 48.951, Math.toRadians(180 - 15)), false);
                } else if (elapsed7 < 1.5) {
                    follower.holdPoint(new Pose(1.111, 48.951, Math.toRadians(180 + 15)), false);
                } else if (elapsed7 < 2.0) {
                    follower.holdPoint(new Pose(1.111, 48.951, Math.toRadians(180 - 15)), false);
                } else {
                    follower.holdPoint(new Pose(1.111, 48.951, Math.toRadians(180)), false);
                }

                if (elapsed7 > 2.5) {
                    timer.resetTimer();
                    setPathState(16);
                }
                break;
            case 16: // Pre-Move Pause: Wait 800ms before Path 8
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path8);
                    setPathState(17);
                }
                break;
            case 17: // Wait for Path 8 travel: stop harvester after 50% distance
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
            case 19: // Wait for Path 9 to reach destination, then begin prime for final Shot 3
                if (!follower.isBusy()) {
                    startLoading();
                    timer.resetTimer();
                    setPathState(901);
                }
                break;
            case 901: // Loading Shot 3: Wait 1.5 seconds
                if (timer.getElapsedTimeSeconds() > 1.5) {
                    startShooting(); // Firing Shot 3
                    timer.resetTimer();
                    setPathState(902);
                }
                break;
            case 902: // Firing Step: Final shot for 0.5 seconds
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    stopShooting();
                    timer.resetTimer();
                    setPathState(903);
                }
                break;
            case 903: // Post-Shot Pause: Brief cooling/stabilization delay
                if (timer.getElapsedTimeSeconds() > 0.25) {
                    timer.resetTimer();
                    setPathState(21);
                }
                break;
            case 21: // Pre-Move Pause: Wait 800ms before final drive to park (Path 10)
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path10);
                    setPathState(22);
                }
                break;
            case 22: // Wait for final park movement to finish
                if (!follower.isBusy()) {
                    setPathState(-1); // Sequence complete
                }
                break;
        }
    }

    /** Updates the current pathState variable. */
    public void setPathState(int state) {
        pathState = state;
    }
}
