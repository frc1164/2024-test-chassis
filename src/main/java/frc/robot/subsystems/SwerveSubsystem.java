package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SerialPort;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);
    private final Pose2d poseThis = new Pose2d();
    private final SwerveModulePosition[] Position = { frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition() };
    private final SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            new Rotation2d(0), Position, poseThis);

    // Create two new SimpleMotorFeedforwards (one right and one left) with gains
    // kS, kV, and kA from SysID characterization
    private SimpleMotorFeedforward feedforwardRight = new SimpleMotorFeedforward(DriveConstants.kSRight,
            DriveConstants.kVRight, DriveConstants.kARight);
    private SimpleMotorFeedforward feedforwardLeft = new SimpleMotorFeedforward(DriveConstants.kSLeft,
            DriveConstants.kVLeft, DriveConstants.kALeft);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4848, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading()); // Add (-) here
    }

    public Pose2d getPose() {
        return odometer.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        SwerveModulePosition[] state = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        odometer.resetPosition(getRotation2d(), state, pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        return states;
    }

    @Override
    public void periodic() {
        getChassisPitchError();

        SwerveModulePosition[] state = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        odometer.update(getRotation2d(), state);
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Rotation", getPose().getRotation().toString());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Roll", gyro.getRoll());
        // SmartDashboard.putNumber("Pitch Rate", gyro.getRawGyroX());
        // SmartDashboard.putNumber("Yaw Rate", gyro.getRawGyroY());
        // SmartDashboard.putNumber("Roll Rate", gyro.getRawGyroZ());
        // SmartDashboard.putNumber("X Acceleration", gyro.getWorldLinearAccelX());
        // SmartDashboard.putNumber("Y Acceleration", gyro.getWorldLinearAccelY());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0], feedforwardLeft);
        frontRight.setDesiredState(desiredStates[1], feedforwardRight);
        backLeft.setDesiredState(desiredStates[2], feedforwardLeft);
        backRight.setDesiredState(desiredStates[3], feedforwardRight);
    }

    public float getChassisPitch() {
        return gyro.getPitch();
    }

    public float getChassisPitchError() {
        double pitch = (double) gyro.getPitch();
        SmartDashboard.putNumber("Pitch", pitch);
        pitch = pitch * Math.PI / 180;
        double g = 1.0;
        double a = gyro.getWorldLinearAccelX();
        pitch = pitch - Math.asin(((Math.sin(pitch) * 2.0 * g * Math.cos(pitch) + a)
                / Math.sqrt(Math.pow(g, 2.0) + Math.pow(Math.sin(pitch), 2.0) + 2.0 * g * Math.sin(pitch)
                        + Math.pow(g, 2.0) * Math.pow(Math.cos(pitch), 2.0)))
                * (180.0 / Math.PI));
        SmartDashboard.putNumber("Corrected Pitch", pitch);
        return (float) pitch;
    }
}