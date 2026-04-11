package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.constants;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(constants.ROBOT_MASS)
            .centripetalScaling(0)
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.01, 0.03))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.15963819, 0.001356))

//            .forwardZeroPowerAcceleration(-70.0)
//            .lateralZeroPowerAcceleration(-75.9)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.005, 0.03))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0))

//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.5,0,0.01,0))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.003, 0, 0, 0.6, 0.09));
    ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(84.6)
            .yVelocity(65.4)
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(constants.FORWARD_OFFSET)
            .strafePodX(constants.LATERAL_OFFSET)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .yawScalar(1.000855)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
