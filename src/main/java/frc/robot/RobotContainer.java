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
import frc.robot.commands.DriveAndPickupNoteCmd;

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
                                () -> !m_driveController.getRawButton(5))); // LB

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
                Trigger OPaButton = m_controller.a();
                Trigger OPbButton = m_controller.b();
                Trigger OPyButton = m_controller.y();
                Trigger OPxButton = m_controller.x();
                Trigger OPlBumper = m_controller.leftBumper();
                Trigger OPrBumper = m_controller.rightBumper();
               

                //LB = Short
                //RB = Stow
                //uDPad = Pickup
                //dDPad = Climb Back
                //lDPad = Climb Left
                //rDPad = Climb Right
                //A = Amp
                //X = Hi
                //Y = Mid
                //B = Low


                Trigger OPlDPad = m_controller.povLeft();
                Trigger OPrDPad = m_controller.povRight();
                Trigger OPuDPad = m_controller.povUp();
                Trigger OPdDPad = m_controller.povDown();
                //OPuDPad.onTrue(new Pickup(m_IPFSSub));
                //OPuDPad.onTrue(new InstantCommand(() -> m_Lift.setLiftSetpoint(LiftConstants.PickupHeight)));
                OPuDPad.whileTrue(new DriveAndPickupNoteCmd(swerveSubsystem));


                

                new JoystickButton(m_driveController, 1).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));



                // Press and hold the B button to Pathfind to Roughly Source. Releasing button should cancel the command
                OPbButton.whileTrue(AutoBuilder.pathfindToPose(
                        new Pose2d(15.75, 1.73, Rotation2d.fromDegrees(0)), 
                        new PathConstraints(
                          1.0, 1.0, 
                          Units.degreesToRadians(180), Units.degreesToRadians(270)
                        ), 
                        0, 
                        2.0
                      ));

                // Press and hold the Y button to Pathfind to (1.83, 3.0, 0 degrees). Releasing button should cancel the command
                OPyButton.whileTrue(AutoBuilder.pathfindToPose(
                        new Pose2d(2.88, 6.99, Rotation2d.fromDegrees(0)), 
                        new PathConstraints(
                          1.0, 1.0, 
                          Units.degreesToRadians(180), Units.degreesToRadians(270)
                        ), 
                        0, 
                        2.0
                      ));

                //Press and hold the X button to Pathfind to the start of the "AMP-Path" path. Releasing the button should cancel the command
                OPaButton.whileTrue(AutoBuilder.pathfindThenFollowPath(
                        PathPlannerPath.fromPathFile("AMP-path"), 
                        new PathConstraints(1.0, 1.0, Units.degreesToRadians(180), Units.degreesToRadians(270)), 
                        0.0));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}