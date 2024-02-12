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
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.Drive.SwerveSubsystem;

import frc.robot.commands.AutoIntakeCmd;
import frc.robot.commands.AutoShootCmd;
import frc.robot.commands.RunClimberCmd;
import frc.robot.commands.ManualIntakeCmd;
import frc.robot.commands.SwerveCmd;


public class RobotContainer {
  private final CommandPS4Controller controller1 = new CommandPS4Controller(ControllerConstants.CONTROLLER_1_PORT);
  private final CommandPS4Controller controller2 = new CommandPS4Controller(ControllerConstants.CONTROLLER_2_PORT);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();

  private final SwerveCmd joystickSwerve = new SwerveCmd(
      swerveSubsystem, 
      () -> -controller1.getLeftY() * DriveConstants.MAX_DRIVE_SPEED, 
      () -> -controller1.getLeftX() * DriveConstants.MAX_DRIVE_SPEED, 
      () -> -controller1.getRightX() * DriveConstants.MAX_SET_ROTATION_SPEED,
      controller1.L1());
  private final AutoShootCmd autoRunShooterCmd = new AutoShootCmd(swerveSubsystem, pivotSubsystem, indexerSubsystem, shooterSubsystem, limelightSubsystem);
  private final AutoIntakeCmd autoIntakeAlignCmd = new AutoIntakeCmd(swerveSubsystem, intakeSubsystem, indexerSubsystem, limelightSubsystem, true);
  private final RunClimberCmd runClimberCmd = new RunClimberCmd(climberSubsystem, () -> controller2.getLeftY());
  private final ManualIntakeCmd runIntakeCmd = new ManualIntakeCmd(intakeSubsystem, indexerSubsystem, () -> controller2.getRightY());

  public static ShuffleboardTab robotStatus = Shuffleboard.getTab("Robot");

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("Auto Shoot", new AutoShootCmd(swerveSubsystem, pivotSubsystem, indexerSubsystem, shooterSubsystem, limelightSubsystem));
    NamedCommands.registerCommand("Auto Intake", new AutoIntakeCmd(swerveSubsystem, intakeSubsystem, indexerSubsystem, limelightSubsystem, true));

    configureBindings();

    swerveSubsystem.setDefaultCommand(joystickSwerve);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chosoer", autoChooser);
  }

  private void configureBindings() {
    // Driver Controls
    controller1.button(0).debounce(ControllerConstants.DEBOUNCE_TIME).onTrue(autoRunShooterCmd);
    controller1.button(1).debounce(ControllerConstants.DEBOUNCE_TIME).onTrue(autoIntakeAlignCmd);

    controller1.button(2).debounce(ControllerConstants.DEBOUNCE_TIME).whileTrue(runIntakeCmd);

    // Operator Controls
    controller2.L2().debounce(ControllerConstants.DEBOUNCE_TIME).whileTrue(runClimberCmd);
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
