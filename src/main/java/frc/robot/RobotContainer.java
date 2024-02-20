package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AprilTagAlignCmd;
import frc.robot.commands.NoteAlignCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import java.util.HashMap;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

        SendableChooser<Command> m_chooser = new SendableChooser<>();
        private final SendableChooser<Command> autoChooser;

        private final Joystick m_driveController = new Joystick(OIConstants.kDriverControllerPort);
       // private final XboxController m_drivedriveController = new XboxController(OIConstants.kDriverControllerPort);
        private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

        // Pathfinding stuff
        private final Pose2d targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));
        private final PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
        private final Command pathfindingCommand;

        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> m_driveController.getRawAxis(OIConstants.kDriverYAxis),
                                () -> m_driveController.getRawAxis(OIConstants.kDriverXAxis),
                                () -> -m_driveController.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !m_driveController.getRawButton(0)));

                // Build an auto chooser. This will use Commands.none() as the default option.
                autoChooser = AutoBuilder.buildAutoChooser();
                pathfindingCommand = AutoBuilder.pathfindToPose(
                        targetPose,
                        constraints,
                        0.0, // Goal end velocity in meters/sec
                        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                );

                // Another option that allows you to specify the default auto by its name
                // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

                SmartDashboard.putData("Auto Chooser", autoChooser);
                configureButtonBindings();
        }

        private void configureButtonBindings() {

                // Sets buttons
                Trigger aButton = m_controller.a();
                Trigger bButton = m_controller.b();
                Trigger yButton = m_controller.y();
                Trigger xButton = m_controller.x();
                Trigger lBumper = m_controller.leftBumper();
                Trigger rBumper = m_controller.rightBumper();
                Trigger lDPad = m_controller.povLeft();
                Trigger rDPad = m_controller.povRight();
                Trigger uDPad = m_controller.povUp();
                Trigger dDPad = m_controller.povDown();

                aButton.whileTrue(new AprilTagAlignCmd(swerveSubsystem));
                bButton.whileTrue(new NoteAlignCmd(swerveSubsystem));

                //Example binding for PathFinding to a specific pose
                //NOTE: ALL NUMBERS ARE BOGUS!!! Change before attempting!!!
                yButton.whileTrue(AutoBuilder.pathfindToPose(
                        new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
                        new PathConstraints(
                          4.0, 4.0, 
                          Units.degreesToRadians(360), Units.degreesToRadians(540)
                        ), 
                        0, 
                        2.0
                      ));

                //Example binding for Pathfinding to the Start Point of a specific path TEST, and then run the Path
                //NOTE: ALL NUMBERS ARE BOGUS!!! Change before attempting!!!
                xButton.whileTrue(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("TEST"), 
                        new PathConstraints(4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)), 
                        0.0)
                      );

                //Example for putting up a button on the SmartDashboard to launch Pathfinding to a specific Pose
                //NOTE: ALL NUMBERS ARE BOGUS!!! Change before attempting!!!
                SmartDashboard.putData("Pathfind to SOURCE", AutoBuilder.pathfindToPose(
                        new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
                        new PathConstraints(
                          4.0, 4.0, 
                          Units.degreesToRadians(360), Units.degreesToRadians(540)
                        ), 
                        0, 
                        2.0
                      ));

                /*
                 * new JoystickButton(driverJoytick, 2).whenPressed(() ->
                 * swerveSubsystem.zeroHeading());
                 */

                new JoystickButton(m_driveController, 1).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}