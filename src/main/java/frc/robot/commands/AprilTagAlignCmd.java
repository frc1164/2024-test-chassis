// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AprilTagAlignCmd extends Command {
  private final SwerveSubsystem m_swerveSubsystem;
  private final PIDController lateraPidController;
  private final NetworkTable limelighNetworkTable;

  private double tv, tx, ty, ta;

  /** Creates a new AprilTagAlignCmd. */
  public AprilTagAlignCmd(SwerveSubsystem swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.lateraPidController = new PIDController(0.05, 0, 0);

    this.limelighNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");

    lateraPidController.setSetpoint(0);

    addRequirements(this.m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelighNetworkTable.getEntry("pipeline").setNumber(0);
    SmartDashboard.putBoolean("AprilTagAlign", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds;

    tv = limelighNetworkTable.getEntry("tv").getDouble(0);
    tx = limelighNetworkTable.getEntry("tx").getDouble(0);
    ty = limelighNetworkTable.getEntry("ty").getDouble(0);
    ta = limelighNetworkTable.getEntry("ta").getDouble(0);
  
    chassisSpeeds = new ChassisSpeeds(0, lateraPidController.calculate(tx), 0);

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    // Output each module states to wheels
    m_swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("AprilTagAlign", false);
    limelighNetworkTable.getEntry("pipeline").setNumber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(tx <= 2) {
      return true;
    }
    return false;
  }
}
