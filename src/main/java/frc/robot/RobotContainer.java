package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Drive.SwerveSubsystem;
import frc.robot.commands.AutoIntakeCmd;
import frc.robot.commands.AutoShootCmd;
import frc.robot.commands.ManualShootCmd;
import frc.robot.commands.RunClimberCmd;
import frc.robot.commands.RunIntakeCmd;
import frc.robot.commands.SwerveCmd;


public class RobotContainer {
  private final CommandPS4Controller controller_1 = new CommandPS4Controller(ControllerConstants.CONTROLLER_1_PORT);
  private final CommandPS4Controller controller_2 = new CommandPS4Controller(ControllerConstants.CONTROLLER_2_PORT);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final SwerveCmd joystickSwerve = new SwerveCmd(
      swerveSubsystem, 
      () -> -controller_1.getLeftY() * DriveConstants.MAX_DRIVE_SPEED, 
      () -> -controller_1.getLeftX() * DriveConstants.MAX_DRIVE_SPEED, 
      () -> -controller_1.getRightX() * DriveConstants.MAX_SET_ROTATION_SPEED,
      controller_1.L1());
  private final AutoShootCmd autoRunShooterCmd = new AutoShootCmd(shooterSubsystem, indexerSubsystem, limelightSubsystem);
  private final ManualShootCmd manualRunShooterCmd = new ManualShootCmd(shooterSubsystem, indexerSubsystem, null, null, null);
  private final AutoIntakeCmd autoIntakeNoAlignCmd = new AutoIntakeCmd(intakeSubsystem, indexerSubsystem, limelightSubsystem, false);
  private final RunClimberCmd runClimberCmd = new RunClimberCmd(climberSubsystem, null);
  private final RunIntakeCmd runIntakeCmd = new RunIntakeCmd(intakeSubsystem, indexerSubsystem, null);

  public static ShuffleboardTab robotStatus = Shuffleboard.getTab("Robot");

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("Auto Shoot", new AutoShootCmd(shooterSubsystem, indexerSubsystem, limelightSubsystem));
    NamedCommands.registerCommand("Auto Intake", new AutoIntakeCmd(intakeSubsystem, indexerSubsystem, limelightSubsystem, true));

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chosoer", autoChooser);
  }

  private void configureBindings() {}


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
