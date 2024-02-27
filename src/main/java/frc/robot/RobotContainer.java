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
import frc.robot.LEDs;
import frc.robot.Constants.LEDConstants.ledMode;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AprilTagAlignCmd;
import frc.robot.commands.NoteAlignCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

        private final SendableChooser<Command> autoChooser;


        protected final static SendableChooser<ledMode> LED_Chooser=new SendableChooser<>();


        public final LEDs m_LEDs;

        private final Joystick m_driveController = new Joystick(OIConstants.kDriverControllerPort);
       // private final XboxController m_drivedriveController = new XboxController(OIConstants.kDriverControllerPort);
        private final CommandXboxController m_controller = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

        public RobotContainer() {
                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> m_driveController.getRawAxis(OIConstants.kDriverYAxis),
                                () -> m_driveController.getRawAxis(OIConstants.kDriverXAxis),
                                () -> -m_driveController.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !m_driveController.getRawButton(0)));

                 //Register named commands
                NamedCommands.registerCommand("NoteAlignedCmd", new NoteAlignCmd(swerveSubsystem));
                NamedCommands.registerCommand("AprilTagAlignCmd", new AprilTagAlignCmd(swerveSubsystem));

                // Build an auto chooser. This will use Commands.none() as the default option.
                autoChooser = AutoBuilder.buildAutoChooser();

                // Another option that allows you to specify the default auto by its name
                // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

                SmartDashboard.putData("Auto Chooser", autoChooser);
                configureButtonBindings();


                LED_Chooser.setDefaultOption("RED", ledMode.RED);
                LED_Chooser.addOption("BLUE", ledMode.BLUE);
                LED_Chooser.addOption("PURPLE", ledMode.PURPLE);
                LED_Chooser.addOption("GREEN", ledMode.GREEN);
                LED_Chooser.addOption("RAINBOW", ledMode.RAINBOW);
                LED_Chooser.addOption("YELLOW", ledMode.YELLOW);
                LED_Chooser.addOption("TEAM", ledMode.TEAM);
                LED_Chooser.addOption("ALLIANCE", ledMode.ALLIANCE);
          
                SmartDashboard.putData("LED COLORS", LED_Chooser);

                m_LEDs = new LEDs();

        }

        private void configureButtonBindings() {

                // Sets buttons for Operator controller
                Trigger operator_aButton = m_controller.a();
                Trigger operator_bButton = m_controller.b();
                Trigger operator_yButton = m_controller.y();
                Trigger operator_xButton = m_controller.x();
                Trigger operator_lBumper = m_controller.leftBumper();
                Trigger operator_rBumper = m_controller.rightBumper();
                Trigger operator_lDPad = m_controller.povLeft();
                Trigger operator_rDPad = m_controller.povRight();
                Trigger operator_uDPad = m_controller.povUp();
                Trigger operator_dDPad = m_controller.povDown();

                operator_aButton.onTrue(new AprilTagAlignCmd(swerveSubsystem));
                // operator_bButton.onTrue(new NoteAlignCmd(swerveSubsystem));

                // Set button bindings for Driver controller. It would probably be better to implement this as an actual XBox controller.
                // Meanshile, here is the XBox controller to JoystickButton mapping. If we are gonna do it like this, these numbers really should be defined in Constants!!!
                //0 = A
                //1 = B 
                //2 = X
                //3 = Y
                //4 = Left bumper
                //5 = Right bumper 

                // Press the B button to zero the heading
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

                new JoystickButton(m_driveController, 1).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

                // Press and hold the B button to Pathfind to (1.83, 3.0, 0 degrees). Releasing button should cancel the command
                operator_bButton.whileTrue(AutoBuilder.pathfindToPose(
                        new Pose2d(15.75, 1.73, Rotation2d.fromDegrees(0)), 
                        new PathConstraints(
                          1.0, 1.0, 
                          Units.degreesToRadians(180), Units.degreesToRadians(270)
                        ), 
                        0, 
                        2.0
                      ));

                // Press and hold the Y button to Pathfind to (1.83, 3.0, 0 degrees). Releasing button should cancel the command
                operator_yButton.whileTrue(AutoBuilder.pathfindToPose(
                        new Pose2d(2.88, 6.99, Rotation2d.fromDegrees(0)), 
                        new PathConstraints(
                          1.0, 1.0, 
                          Units.degreesToRadians(180), Units.degreesToRadians(270)
                        ), 
                        0, 
                        2.0
                      ));

                //Press and hold the X button to Pathfind to the start of the "AMP-Path" path. Releasing the button should cancel the command
                operator_xButton.whileTrue(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("AMP-path"), 
                        new PathConstraints(1.0, 1.0, Units.degreesToRadians(180), Units.degreesToRadians(270)), 
                        0.0));

                //These SmartDashboard buttons do the same thing as the two above button bindings. They can be safely deleted once the controller button bindings work.
                SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
                        new Pose2d(1.83, 3.0, Rotation2d.fromDegrees(0)), 
                        new PathConstraints(
                          1.0, 1.0, 
                          Units.degreesToRadians(180), Units.degreesToRadians(270)
                        ), 
                        0, 
                        2.0
                      ));

                SmartDashboard.putData("Pathfind to AMP Path", AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("AMP-path"), 
                        new PathConstraints(1.0, 1.0, Units.degreesToRadians(180), Units.degreesToRadians(270)), 
                        0.0)
                      );       
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}