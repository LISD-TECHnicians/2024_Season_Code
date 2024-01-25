package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

import frc.robot.Constants.ControllerConstants;

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.AutoIntakeCmd;
import frc.robot.commands.AutoShootCmd;
import frc.robot.commands.ManualShootCmd;
import frc.robot.commands.RunClimberCmd;
import frc.robot.commands.RunIntakeCmd;

public class RobotContainer {
  private final CommandPS4Controller controller_1 = new CommandPS4Controller(ControllerConstants.CONTROLLER_1_PORT);
  private final CommandPS4Controller controller_2 = new CommandPS4Controller(ControllerConstants.CONTROLLER_2_PORT);

  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  private final AutoShootCmd autoRunShooterCmd = new AutoShootCmd(shooterSubsystem, indexerSubsystem, limelightSubsystem);
  private final ManualShootCmd manualRunShooterCmd = new ManualShootCmd(shooterSubsystem, indexerSubsystem, null, null, null);
  private final AutoIntakeCmd autoIntakeNoAlignCmd = new AutoIntakeCmd(intakeSubsystem, indexerSubsystem, limelightSubsystem, false);
  private final RunClimberCmd runClimberCmd = new RunClimberCmd(climberSubsystem, null);
  private final RunIntakeCmd runIntakeCmd = new RunIntakeCmd(intakeSubsystem, null);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}


  public Command getAutonomousCommand() {
    return null;
  }
}
