package org.firstinspires.ftc.teamcode;

// Import required classes for FTC OpMode, hardware control, and Pedro Pathing
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
 * BLUE_Auto_LONG autonomous OpMode.
 * This class handles the autonomous sequence for the Blue side "Long" position.
 * Features a 20-second initial wait, high-speed movement, and integrated mechanism control.
 */
@Autonomous(name = "BLUE_Auto_LONG", group = "Autonomous")
@Configurable // Enables configuration through the Panels dashboard
public class BLUE_Auto_LONG extends OpMode {
    // Manages telemetry data sent to the dashboard
    private TelemetryManager panelsTelemetry;
    // Core object for Pedro Pathing movement and localization
    public Follower follower;
    // Current step in the autonomous state machine
    private int pathState;
    // Instance of the inner Paths class containing trajectory definitions
    private Paths paths;
    // Timer object for handling state durations and delays
    private Timer timer;

    // Motor objects for robot mechanisms
    private DcMotor CL_1, CL_2, HAR_MTR;

    /**
     * Called once when the robot is initialized from the Driver Station.
     */
    @Override
    public void init() {
        // Initialize telemetry manager
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        // Setup the path follower with hardware and constants
        follower = Constants.createFollower(hardwareMap);
        
        // Initialize motors using hardware configuration names
        CL_1 = hardwareMap.get(DcMotor.class, "CL_1");
        CL_2 = hardwareMap.get(DcMotor.class, "CL_2");
        HAR_MTR = hardwareMap.get(DcMotor.class, "HAR_MTR");

        // Set motors to hold position when power is 0 (BRAKE mode)
        CL_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CL_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HAR_MTR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define motor rotation directions
        CL_1.setDirection(DcMotorSimple.Direction.FORWARD);
        CL_2.setDirection(DcMotorSimple.Direction.FORWARD);
        HAR_MTR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set the robot's starting location on the field coordinate system
        follower.setStartingPose(new Pose(56.000, 8.000, Math.toRadians(90)));
        // Initialize path definitions
        paths = new Paths(follower);
        // Start the autonomous timer
        timer = new Timer();

        // Telemetry update
        panelsTelemetry.debug("Status", "Initialized BLUE_Auto_LONG");
        panelsTelemetry.update(telemetry);
    }

    /**
     * Loops continuously while the OpMode is active.
     */
    @Override
    public void loop() {
        // Update localization and path following logic (REQUIRED)
        follower.update();
        // Check for state transitions in the autonomous sequence
        autonomousPathUpdate();

        // Output current robot state and pose to the telemetry dashboard
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    /** Helper method to start mechanism for loading balls into the shooter. */
    public void startLoading() {
        CL_1.setPower(0.5);
        CL_2.setPower(-0.5);
    }

    /** Helper method to start firing mechanism. */
    public void startShooting() {
        CL_1.setPower(-1.0);
        CL_2.setPower(1.0);
    }

    /** Helper method to stop all mechanism motors. */
    public void stopShooting() {
        CL_1.setPower(0);
        CL_2.setPower(0);
    }

    /** Inner class defining all pathing trajectories for this autonomous. */
    public static class Paths {
        public PathChain Path1, Path2, Path3;

        public Paths(Follower follower) {
            // Define constraints for travel (1.0 = 100% max drivetrain power)
            PathConstraints fastConstraints = new PathConstraints(1.0, 30, 1, 1);

            // Path 1: Drive from start to intermediate field position
            Path1 = follower.pathBuilder(fastConstraints)
                .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(59.191, 250.990)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(140))
                .build();

            // Path 2: Drive from end of Path 1 to the shooting location
            Path2 = follower.pathBuilder(fastConstraints)
                .addPath(new BezierLine(new Pose(59.191, 250.990), new Pose(26.932, 119.625)))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(140))
                .build();

            // Path 3: Drive from shooting location to final park position
            Path3 = follower.pathBuilder(fastConstraints)
                .addPath(new BezierLine(new Pose(26.932, 119.625), new Pose(15.176, 83.235)))
                .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                .build();
        }
    }

    /**
     * Autonomous state machine logic.
     * Manages sequence: Delay -> Path 1 -> Pause -> Path 2 -> Pause -> Mechanisms -> Pause -> Path 3
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Step 0: Idle for initial 20-second delay
                if (timer.getElapsedTimeSeconds() > 20.0) {
                    follower.followPath(paths.Path1); // Begin first move
                    setPathState(1);
                }
                break;
            case 1: // Step 1: Wait for Path 1 to complete
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(7); // Go to post-path pause
                }
                break;
            case 7: // Step 7: Pause for 800ms
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path2); // Begin second move
                    setPathState(2);
                }
                break;
            case 2: // Step 2: Wait for Path 2 to complete
                if (!follower.isBusy()) {
                    timer.resetTimer();
                    setPathState(8); // Go to pre-mechanism pause
                }
                break;
            case 8: // Step 8: Pause for 800ms before mechanism start
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    startLoading(); // Activate mechanisms
                    timer.resetTimer();
                    setPathState(3);
                }
                break;
            case 3: // Step 3: Load balls for 1.5 seconds
                if (timer.getElapsedTimeSeconds() > 1.5) {
                    startShooting(); // Activate firing
                    timer.resetTimer();
                    setPathState(4);
                }
                break;
            case 4: // Step 4: Fire for 0.5 seconds
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    stopShooting(); // Deactivate all mechanisms
                    timer.resetTimer();
                    setPathState(5);
                }
                break;
            case 5: // Step 5: Post-firing pause for 800ms
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    follower.followPath(paths.Path3); // Begin final move
                    setPathState(6);
                }
                break;
            case 6: // Step 6: Wait for final move completion
                if (!follower.isBusy()) {
                    setPathState(-1); // Stop state machine
                }
                break;
        }
    }

    /** Updates the current pathState index. */
    public void setPathState(int state) {
        pathState = state;
    }
}
