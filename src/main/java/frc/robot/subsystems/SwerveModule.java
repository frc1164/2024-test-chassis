package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    /* private final AnalogInput absoluteEncoder; */ 
    private final CANcoder absoluteEncoder;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderId, "rio");
                
        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        driveMotor.setIdleMode(IdleMode.kBrake);

        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        turningMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        //turningEncoder.setPositionConversionFactor(turningEncoder.getCountsPerRevolution()/(2*Math.PI));
        //turningEncoder.setVelocityConversionFactor(turningEncoder.getCountsPerRevolution()/(2*Math.PI));
        //driveEncoder.setPositionConversionFactor(driveEncoder.getCountsPerRevolution()/(2*Math.PI));
        //driveEncoder.setVelocityConversionFactor(driveEncoder.getCountsPerRevolution()/(2*Math.PI));

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        /* double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V(); */ 
        double angle = absoluteEncoder.getAbsolutePosition().getValue() * 2 * Math.PI + Math.PI / 2;
        /* angle *= 2.0 * Math.PI; */
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state, SimpleMotorFeedforward feedforward) {
        if (Math.abs(feedforward.calculate(state.speedMetersPerSecond)) < 0.1) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        /* driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond); Original line */
        driveMotor.setVoltage(feedforward.calculate(state.speedMetersPerSecond));
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
    }
    
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
