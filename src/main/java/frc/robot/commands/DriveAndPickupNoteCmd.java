// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveAndPickupNoteCmd extends Command {
  private final SwerveSubsystem m_swerveSubsystem;
  private final NetworkTable limelighNetworkTable;
  private final PIDController lateralPidController, longitudinalPidController, rotationalPidController;
  private double previousPipeline;
  private double tv, tx, ty, ta, lastYaw, lateralDistanceToNote, distanceFromNote, predictedLateralDistanceTraveled;
  private double degreesToRotate;
  private double tOld, tNew;
  private boolean detectNotes, moveToNote;

  // Change these values as needed
  private double noteSizeThreshold = 0.5;                 // Minimum size of the NOTE for the robot to move to and pickup.
  private double overshootDistance = 0.5;                 // This is how many meters will be added to the distance to try to run over the NOTE.
  private double LimeLightHeight = 0.71;                  // The LimeLight distance off the floor in meters, used to estimate distance to a NOTE.
  private double minimumNoteAngle = -21.5;                // The minimum angle for the LimeLight to calculate distance to the NOTE. Below this, the robot will estimate.
  private String limelighNetworkTableName = "limelight";  // Should be limelight-pickup later
  private double translationP = 0.05;                     // P can be 0.05 after initial testing
  private double translationI = 0;
  private double translationD = 0;

  private double rotationP = 0.004;
  private double rotationI = 0.002;
  private double rotationD = 0.0005;
  //

  /**
   * Select the object detection pipeline (on the Pickup LL)
   * Rotate in place until you see a NOTE or notes
   * Center on and drive over the closest note
   * Eventually automatically pick it up
   */

  /** Creates a new DriveAndPickupNoteCmd. */
  public DriveAndPickupNoteCmd(SwerveSubsystem swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.lateralPidController = new PIDController(translationP, translationI, translationD);
    this.longitudinalPidController = new PIDController(translationP, translationI, translationD);
    this.rotationalPidController = new PIDController(rotationP, rotationI, rotationD);

    this.limelighNetworkTable = NetworkTableInstance.getDefault().getTable(limelighNetworkTableName);

    lateralPidController.setSetpoint(0);
    longitudinalPidController.setSetpoint(0);
    rotationalPidController.setSetpoint(0);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousPipeline = limelighNetworkTable.getEntry("pipeline").getDouble(0);
    limelighNetworkTable.getEntry("pipeline").setNumber(1);
    SmartDashboard.putBoolean("DriveAndPickupNote", true);
    tv = 0;
    tx = 0;
    ty = 0;
    ta = 0;
    degreesToRotate = 360; // Rotate in a full circle to find any NOTES.
    detectNotes = true; // Try to search for NOTES
    moveToNote = false; // Rotate to find NOTES, don't immeaditly try to move to one and pick it up.
    lastYaw = m_swerveSubsystem.getChassisYaw();
    lateralDistanceToNote = 0;
    predictedLateralDistanceTraveled = 0;
    tOld = 0;
    tNew = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Initialize ChassisSpeeds to control robot movement
    ChassisSpeeds chassisSpeeds;

    // If the robot should look for a NOTE, get data from the LimeLight and rotate in place.
    if(detectNotes == true) {
      // Get most recent LimeLight position on a NOTE, if present.
      tv = limelighNetworkTable.getEntry("tv").getDouble(0);
      tx = limelighNetworkTable.getEntry("tx").getDouble(0);
      ty = limelighNetworkTable.getEntry("ty").getDouble(0);
      ta = limelighNetworkTable.getEntry("ta").getDouble(0);

      // Decrease degreesToRotate as the robot completes its search.
      degreesToRotate = degreesToRotate - (lastYaw - m_swerveSubsystem.getChassisYaw());
      lastYaw = m_swerveSubsystem.getPose().getRotation().getDegrees();
      chassisSpeeds = new ChassisSpeeds(0, 0, rotationalPidController.calculate(degreesToRotate));

      // If the robot sees a NOTE that is close and isn't pointed at it, stop the full-circle scan and point the robot at the NOTE.
      if(ta >= noteSizeThreshold && Math.abs(tx) > 3) {
        degreesToRotate = tx;
      }
      
      // If the robot sees a NOTE and is reasonably pointed at it, begin moving towards the NOTE.
      if(ta >= noteSizeThreshold && Math.abs(tx) <= 3) {
        // Guess the distance until the robot encounters the NOTE. Begin moving towards the NOTE.
        if((lateralDistanceToNote == 0 || ty >= minimumNoteAngle) && !moveToNote) {
          distanceFromNote = Math.tan((90 - Math.abs(ty)) * Math.PI/180) * LimeLightHeight;
          moveToNote = true;
        }

        // Prevent from tracking a new NOTE after the current NOTE leaves its field of view.
        if(ty < minimumNoteAngle) {
          degreesToRotate = 0;
          detectNotes = false;
        }
      }
    } 
    else {
      tv = 0;
      tx = 0;
      ty = 0;
      ta = 0;
    chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    }

    // If a NOTE is seen, go there and record timestamps to calculate distance traveled.
    if(moveToNote == true) {        
      tOld = tNew;
      tNew = System.nanoTime() / Math.pow(10, 9);
      SmartDashboard.putNumber("tnew", tNew);
      // Predict distance traveled based on meters/second multiplied by time traveling at the velocity.
      if(tOld != 0 && tNew != 0) {
        predictedLateralDistanceTraveled = predictedLateralDistanceTraveled + (m_swerveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond * (tNew - tOld));
        lateralDistanceToNote = distanceFromNote + overshootDistance - predictedLateralDistanceTraveled;
        SmartDashboard.putNumber("predictedLateralDistanceTraveled", predictedLateralDistanceTraveled);
        chassisSpeeds = new ChassisSpeeds(longitudinalPidController.calculate(-lateralDistanceToNote), lateralPidController.calculate(tx), 0);
      }
    }

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
    // Output each module states to wheels
    m_swerveSubsystem.setModuleStates(moduleStates);
    SmartDashboard.putNumber("lateralDistanceToNote", lateralDistanceToNote);
    SmartDashboard.putNumber("distanceFromNote", distanceFromNote);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //limelighNetworkTable.getEntry("pipeline").setNumber(previousPipeline);
    SmartDashboard.putBoolean("DriveAndPickupNote", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(lateralDistanceToNote) <= 0.15 && Math.abs(lateralDistanceToNote) > 0) {  // Add or statement for NOTE in pickup
        return true;
    }
    return false;
  }
}
