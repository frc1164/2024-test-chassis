package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;


public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1/6.75;
        public static final double kTurningMotorGearRatio = 7.00 / 150.00;//150.00 / 7.00;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio *  kWheelDiameterMeters * Math.PI;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2.0 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(24.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(24.75);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), /* Left front */
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), /* Right front */
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), /* Left rear */
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); /* Right rear */

        public static final int kFrontLeftDriveMotorPort = 10;
        public static final int kBackLeftDriveMotorPort = 40;
        public static final int kFrontRightDriveMotorPort = 20;
        public static final int kBackRightDriveMotorPort = 30;

        public static final int kFrontLeftTurningMotorPort = 11;
        public static final int kBackLeftTurningMotorPort = 41;
        public static final int kFrontRightTurningMotorPort = 21;
        public static final int kBackRightTurningMotorPort = 31;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 12;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 42;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 22;
        public static final int kBackRightDriveAbsoluteEncoderPort = 32;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        /*public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 109.86 * Math.PI/180.0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 240.47 * Math.PI/180.0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 334.86 * Math.PI/180.0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 191.16 * Math.PI/180.0;*/
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 167.695 * Math.PI/180.0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 45.0 * Math.PI/180.0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 284.15 * Math.PI/180.0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 245.25 * Math.PI/180.0;


        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        /* (Also Incorrect) Feedforward constants from SysID run on 2/19/24 */
        //public static final double kSLeft = 0.95317;
        //public static final double kVLeft = 0.26112;
        //public static final double kALeft = 1.0539;

        //public static final double kSRight = 0.48604;
        //public static final double kVRight = 1.7619;
        //public static final double kARight = 0.66864;

        /* (Incorrect) Feedforward constants from Daedalus */
         public static final double kSLeft = 0.097576;
         public static final double kVLeft = 2.6933;
         public static final double kALeft = 0.26236;

         public static final double kSRight = 0.099437;
         public static final double kVRight = 2.6173;
         public static final double kARight = 0.11195;

        // Drive/Rotation gain
        public static final double kRotGain = 2;
        public static final double kDriveGain = 3;
    }

    public static final class LimeLightConstants {
        public static final String kllShoot = "Limelight-shoot";
        public static final String kllPickup = "Limelight-pickup";
        public static final int kAprilTagPipeline = 0;
        public static final int kNeuralNetworkPipeline = 1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0.1;
        public static final double kPYController = 0.1;
        public static final double kPThetaController = 10;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 5;

        public static final double kDeadband = 0.1;
    }

    public static class OperatorConstants {
      public static final int kOperatorControllerPort = 1;
    }

    public static class VisionConstants {
        public static final String kPickupLimelightNetworkTableName = "limelight-pickup";
        public static final String kShootLimelightNetworkTableName = "limelight-shoot";
    }

    public static class LEDConstants {
        public static final int statusLEDport = 0;
        public static final int statusLEDlength = 36;
        public static final int ziaLEDport = 1;
        public static final int ziaLEDlength = 48;

        public enum ledMode {
            RED, GREEN, RAINBOW, TEAM, BLUE, PURPLE, YELLOW, ALLIANCE;
          }
    }
}
