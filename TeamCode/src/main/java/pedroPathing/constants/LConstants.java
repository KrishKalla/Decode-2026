package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.pedropathing.localization.localizers.ThreeWheelIMULocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;



public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = .001989436789;
        ThreeWheelIMUConstants.strafeTicksToInches = .001989436789;
        ThreeWheelIMUConstants.turnTicksToInches = 0.001912;
        ThreeWheelIMUConstants.leftY = 7.35;
        ThreeWheelIMUConstants.rightY = -7.35;
        ThreeWheelIMUConstants.strafeX = -1;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "backLeft";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "backRight";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "frontRight";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        ThreeWheelIMULocalizer.useIMU = false;
    }
}



