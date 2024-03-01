package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AmpShootCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public AmpShootCmd(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, IndexerSubsystem indexerSubsystem, 
      ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(intakeSubsystem, pivotSubsystem, indexerSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(PivotConstants.AMP_ANGLE);
    shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_AMP_SPEED);

    if (pivotSubsystem.getAmpReadiness()) {
      indexerSubsystem.setIndexerSpeed(IndexerConstants.INDEXER_DEFAULT_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeSpeed(0);
    indexerSubsystem.setIndexerSpeed(0);
    shooterSubsystem.setShooterSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
