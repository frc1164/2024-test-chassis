package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LEDs;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AprilTagAlignCmd;
import frc.robot.commands.NoteAlignCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import java.util.HashMap;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

        private final SendableChooser<Command> autoChooser;

        private final LEDs m_LEDs;

        private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
        private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
                
                m_LEDs = new LEDs();

                 //Register named commands
                NamedCommands.registerCommand("NoteAlignedCmd", new NoteAlignCmd(swerveSubsystem));
                NamedCommands.registerCommand("AprilTagAlignCmd", new AprilTagAlignCmd(swerveSubsystem));

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

                aButton.onTrue(new AprilTagAlignCmd(swerveSubsystem));
                bButton.onTrue(new NoteAlignCmd(swerveSubsystem));

                /*
                 * new JoystickButton(driverJoytick, 2).whenPressed(() ->
                 * swerveSubsystem.zeroHeading());
                 */
                new JoystickButton(driverJoytick, 2).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}