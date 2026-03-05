package org.firstinspires.ftc.teamcode;

// Import standard FTC SDK and Pedro Pathing classes
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
 * RED_Auto_LONG autonomous OpMode.
 * This class handles the autonomous sequence for the Red side "Long" position.
 * Sequence includes a 20-second initial delay, path following to a shooting location,
 * loading and firing the shooter, and then moving to a final position.
 */
@Autonomous(name = "RED_Auto_LONG", group = "Autonomous")
@Configurable // Annotation for Panels integration to allow on-the-fly configuration
public class RED_Auto_LONG extends OpMode {
    // Instance of TelemetryManager for enhanced dashboard/telemetry display
    private TelemetryManager panelsTelemetry;
    // The main Pedro Pathing follower object that handles path execution
    public Follower follower;
    // Integer variable to track our current position in the autonomous state machine
    private int pathState;
    // Inner class instance that holds all the pre-defined path trajectories
    private Paths paths;
    // Timer object used for managing delays and timed actions within states
    private Timer timer;

    // Hardware motor declarations for accessory mechanisms (shooter and harvester)
    private DcMotor CL_1, CL_2, HAR_MTR;

    /**
     * The init() method is called when the "INIT" button is pressed on the Driver Station.
     */
    @Override
    public void init() {
        // Retrieve the singleton TelemetryManager instance
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        // Create the Follower instance using the configuration defined in the project's Constants class
        follower = Constants.createFollower(hardwareMap);
        
        // Map mechanisms to the configuration names defined in the robot's hardware profile
        CL_1 = hardwareMap.get(DcMotor.class, "CL_1");
        CL_2 = hardwareMap.get(DcMotor.class, "CL_2");
        HAR_MTR = hardwareMap.get(DcMotor.class, "HAR_MTR");

        // Set motors to "BRAKE" mode so they stop immediately when power is set to 0
        CL_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CL_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        HAR_MTR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define the rotation directions for mechanism motors
        CL_1.setDirection(DcMotorSimple.Direction.FORWARD);
        CL_2.setDirection(DcMotorSimple.Direction.FORWARD);
        HAR_MTR.setDirection(DcMotorSimple.Direction.REVERSE);

        // Tell the localizer exactly where the robot is starting on the field (mirrored Red Long coords)
        follower.setStartingPose(new Pose(88.000, 8.000, Math.toRadians(90)));
        // Initialize the paths class which builds all the Bezier trajectories
        paths = new Paths(follower);
        // Start the timer; it begins counting immediately
        timer = new Timer();

        // Log initialization status to the dashboard
        panelsTelemetry.debug("Status", "Initialized RED_Auto_LONG");
        panelsTelemetry.update(telemetry);
    }

    /**
     * The loop() method is called repeatedly while the robot is running.
     */
    @Override
    public void loop() {
        // MUST call update() every loop to process localization and motor powers
        follower.update();
        // Process the autonomous logic transitions
        autonomousPathUpdate();

        // Send diagnostic data to the dashboard for debugging
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    /**
     * Sets mechanism power to move balls into the shooter chamber.
     */
    public void startLoading() {
        CL_1.setPower(0.5);
        CL_2.setPower(-0.5);
    }

    /**
     * Sets mechanism power to fire balls out of the shooter.
     */
    public void startShooting() {
        CL_1.setPower(-1.0);
        CL_2.setPower(1.0);
    }

    /**
     * Stops all mechanism motors.
     */
    public void stopShooting() {
        CL_1.setPower(0);
        CL_2.setPower(0);
    }

    /**
     * Static inner class to define and store all path trajectories for this OpMode.
     */
    public static class Paths {
        // Objects to hold the specific trajectories
        public PathChain Path1, Path2, Path3;

        public Paths(Follower follower) {
            // Define constraints for "full speed" travel (1.0 max power)
            PathConstraints fastConstraints = new PathConstraints(1.0, 30, 1, 1);

            // Path 1: Initial drive across the field. Uses linear interpolation for heading.
            Path1 = follower.pathBuilder(fastConstraints)
                .addPath(new BezierLine(new Pose(88.000, 8.000), new Pose(84.809, 250.990)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(40))
                .build();

            // Path 2: Move to the shooting position.
            Path2 = follower.pathBuilder(fastConstraints)
                .addPath(new BezierLine(new Pose(84.809, 250.990), new Pose(117.068, 119.625)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(40))
                .build();

            // Path 3: Final move to park/end position.
            Path3 = follower.pathBuilder(fastConstraints)
                .addPath(new BezierLine(new Pose(117.068, 119.625), new Pose(128.824, 83.235)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();
        }
    }

    /**
     * The main state machine logic for autonomous.
     * Transitions from one action to the next based on timers or path completion.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Initial State: Wait for 20 seconds before doing anything
                if (timer.getElapsedTimeSeconds() > 20.0) {
                    // Start following Path 1
                    follower.followPath(paths.Path1);
                    setPathState(1);
                }
                break;
            case 1: // Check if Path 1 is done
                if (!follower.isBusy()) {
                    // Reset timer to track the 800ms pause
                    timer.resetTimer();
                    setPathState(7); // Transition to pause state after Path 1
                }
                break;
            case 7: // Pause State: Wait for 800ms
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    // Start following Path 2
                    follower.followPath(paths.Path2);
                    setPathState(2);
                }
                break;
            case 2: // Check if Path 2 is done
                if (!follower.isBusy()) {
                    // Reset timer for the next pause
                    timer.resetTimer();
                    setPathState(8); // Transition to pause state after Path 2
                }
                break;
            case 8: // Pause State: Wait for 800ms before mechanics start
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    // Start the loading mechanism
                    startLoading();
                    timer.resetTimer(); // Reset timer to track the 1.5s loading time
                    setPathState(3);
                }
                break;
            case 3: // Loading State: Wait 1.5 seconds for mechanisms to prime
                if (timer.getElapsedTimeSeconds() > 1.5) {
                    // Start firing mechanisms
                    startShooting();
                    timer.resetTimer(); // Reset timer to track the 0.5s firing duration
                    setPathState(4);
                }
                break;
            case 4: // Firing State: Shoot for 0.5 seconds
                if (timer.getElapsedTimeSeconds() > 0.5) {
                    // Stop mechanisms
                    stopShooting();
                    timer.resetTimer(); // Reset timer for the post-shot pause
                    setPathState(5);
                }
                break;
            case 5: // Post-Shot Pause: Wait 800ms
                if (timer.getElapsedTimeSeconds() > 0.8) {
                    // Start driving to the final position (Path 3)
                    follower.followPath(paths.Path3);
                    setPathState(6);
                }
                break;
            case 6: // Final State: Check if the robot has arrived at its destination
                if (!follower.isBusy()) {
                    setPathState(-1); // Set to -1 to stop the state machine
                }
                break;
        }
    }

    /**
     * Helper method to update the pathState variable.
     * @param state The new state to transition to.
     */
    public void setPathState(int state) {
        pathState = state;
    }
}
