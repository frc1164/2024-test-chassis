package frc.robot.subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.LimelightHelpers;
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
    //private final SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            //new Rotation2d(0), Position, poseThis);

    private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            new Rotation2d(0), Position, poseThis);

    // Create a new Field2d object for plotting pose and initialize LimeLight Network table instances
    private final Field2d m_field = new Field2d();
    private final NetworkTable pickupLLTable = NetworkTableInstance.getDefault().getTable(LimeLightConstants.kllPickup);
    //Uncomment when we get Limelights renamed
    //private final NetworkTable pickupLLTable = NetworkTableInstance.getDefault().getTable(LimeLightConstants.kllPickup);
    private final NetworkTable shooterLLTable = NetworkTableInstance.getDefault().getTable("limelight");
    private double tv;
    private double ta;
    private double tl;
    //private LimelightHelpers.LimelightResults results;
    private LimelightHelpers.PoseEstimate limelightMeasurement
    
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

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
        return Rotation2d.fromDegrees(getHeading());
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

    public void updateValues() {

        //TODO: Maybe switch these to use the limelight helper sub
        //tv = shooterLLTable.getEntry("tv").getDouble(0);
        //ta = shooterLLTable.getEntry("ta").getDouble(0);
        //tl = shooterLLTable.getEntry("tl").getDouble(40);

        //results = LimelightHelpers.getLatestResults("Limelight-shoot");

        limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-pickup")

    }

    public boolean isGoodTarget() {
        return ta >= 0.5  || results.targetingResults.targets_Fiducials.length > 1 && ta>0.4;
    }
    public boolean hasTargets() {
        return tv == 1;
    }

    public Pose2d getVisionEstimatedPose() {

        double[] bot_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double bot_x, bot_y, rotation_z;

        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
            bot_pose = shooterLLTable
                    .getEntry("botpose_wpiblue")
                    .getDoubleArray(new double[6]);
            } else if (alliance.get() == Alliance.Red) {
            bot_pose = shooterLLTable
                    .getEntry("botpose_wpired")
                    .getDoubleArray(new double[6]);
            }
        }

        bot_x = bot_pose[0];
        bot_y = bot_pose[1];
        rotation_z = (bot_pose[5] + 360) % 360;


        return new Pose2d(
                new Translation2d(bot_x, bot_y),
                Rotation2d.fromDegrees(rotation_z));
    }


    public void updatePoseEstimatorWithVisionBotPose() {
        //PoseLatency visionBotPose = m_visionSystem.getPoseLatency();
        Pose2d visionPose = getVisionEstimatedPose();
        // invalid LL data
        //if (visionBotPose.pose2d.getX() == 0.0) {
        //    return;
        //}
        
        if (visionPose.getX() == 0.0) {
            return;
        }

        // distance from current pose to vision estimated pose
        //double poseDifference = m_poseEstimator.getEstimatedPosition().getTranslation()
        //    .getDistance(visionBotPose.pose2d.getTranslation());

        double poseDifference = m_poseEstimator.getEstimatedPosition().getTranslation()
            .getDistance(visionPose.getTranslation());


        if (limelightMeasurement.tagCount > 0) {
            double xyStds;
            double degStds;
            // multiple targets detected
            if (limelightMeasurement.tagCount >= 2) {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and close to estimated pose
            else if (limelightMeasurement.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (limelightMeasurement.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }
            // conditions don't match to add a vision measurement
            else {
                return;
            }

            m_poseEstimator.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
            m_poseEstimator.addVisionMeasurement(visionPose,
                Timer.getFPGATimestamp() - Units.millisecondsToSeconds(limelightMeasurement.latency));
        }
    }



    public double getLatency() {
        return Timer.getFPGATimestamp() - Units.millisecondsToSeconds(tl);

        // maybe need camera_latency?
        // TODO: TEST 
        // return results.targetingResults.latency_capture;

        // TODO: TEST this breaks it for some reason
        // return llresults.targetingResults.latency_pipeline; 
    }

    @Override
    public void periodic() {

        SwerveModulePosition[] positions = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        m_poseEstimator.update(getRotation2d(), positions);

        // The .name() method seems to have been removed from DriverStation.getAlliance. So this needs a switch statement or something
        //SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().name());
        
        // Set the robot pose on the Field2D object
        m_field.setRobotPose(this.getPose());
        SmartDashboard.putData(m_field);

        updateValues();

        SmartDashboard.putBoolean("Vision Identified Tag", hasTargets());
        SmartDashboard.putBoolean("Is good target", isGoodTarget());

        /*

        // if the limelight has a target
        if (hasTargets()) {   //Originally if (hasTargets() && isGoodTarget()). isGoodTarget() never proves true for some reason
            // grab data off network tables and clean it up a bit

            //  Pose2d currentPosition = odometer.getEstimatedPosition();
            Pose2d visionPose = getVisionEstimatedPose();

            // if the data is good, use it to update the pose estimator
            if (visionPose.getX() != 0 && visionPose.getY() != 0) // &&

            // these check if vision is within a meter of our estimated pose otherwise we
            // ignore it
            // Math.abs(currentPosition.getX() - visionPose.getX()) < 1 &&    
            // Math.abs(currentPosition.getY() - visionPose.getY()) < 1) {
      
            // Add the vision measurement to the PoseEstimator
            odometer.addVisionMeasurement(visionPose, getLatency());
                
            }
        */

        updatePoseEstimatorWithVisionBotPose()


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

    // Though related to the LimeLights, these functions are needed for pose esimation.
    
    
    /* This can probably be safely removed
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
    } */
}