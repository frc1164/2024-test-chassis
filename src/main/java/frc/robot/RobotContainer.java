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
import com.pathplanner.lib.path.PathConstraints;

import java.util.HashMap;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

        SendableChooser<Command> m_chooser = new SendableChooser<>();
        private final SendableChooser<Command> autoChooser;
        public final HashMap<String, Command> eventMap = new HashMap<>();

        private final Joystick m_driveController = new Joystick(OIConstants.kDriverControllerPort);
       // private final XboxController m_drivedriveController = new XboxController(OIConstants.kDriverControllerPort);
        private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

        private final Pose2d targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));
        private final PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
        
        private final Command pathfindingCommand = AutoBuilder.pathfindToPose(
                        targetPose,
                        constraints,
                        0.0, // Goal end velocity in meters/sec
                        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                );

        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> m_driveController.getRawAxis(OIConstants.kDriverYAxis),
                                () -> m_driveController.getRawAxis(OIConstants.kDriverXAxis),
                                () -> -m_driveController.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !m_driveController.getRawButton(0)));

                // Build an auto chooser. This will use Commands.none() as the default option.
                autoChooser = AutoBuilder.buildAutoChooser();

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
          //      yButton.whileTrue(new pathfindingCommand());
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