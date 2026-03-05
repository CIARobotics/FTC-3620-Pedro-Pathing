package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .forwardZeroPowerAcceleration(-32.8512731)
            .lateralZeroPowerAcceleration(-75.35681905)
            // Increased P gains (first value) to help reach the target more accurately
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.01, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04, 0, 0.00001, 0.6, 0.01))
            .centripetalScaling(0.0005);

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .xVelocity(77.9215549)
            .yVelocity(49.36982931)
            .leftFrontMotorName("FL_MTR")
            .rightFrontMotorName("FR_MTR")
            .leftRearMotorName("BL_MTR")
            .rightRearMotorName("BR_MTR")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardPodY(101.6)
            .strafePodX(-152.4)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .customEncoderResolution(19.0268)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // Kept at 0.2 power as requested, but slightly increased P gains will help it finish the move.
    public static PathConstraints pathConstraints = new PathConstraints(0.2, 30, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
