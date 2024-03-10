package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;

import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShootCmd extends Command {
  private final PivotSubsystem pivotSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  private final Timer shooterTimer = new Timer();

  public ManualShootCmd(PivotSubsystem pivotSubsystem, IndexerSubsystem indexerSubsystem, 
      ShooterSubsystem shooterSubsystem) {
    this.pivotSubsystem = pivotSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem, pivotSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {
    shooterTimer.restart();
  }

  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(PivotConstants.SHOOT_ANGLE);

    shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_DEFAULT_SPEED);

    if (pivotSubsystem.getIntakeReadiness()) {
      indexerSubsystem.setIndexerSpeed(IndexerConstants.INDEXER_DEFAULT_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.setIndexerSpeed(0.0);
    shooterSubsystem.setShooterSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return shooterTimer.hasElapsed(ShooterConstants.SHOOTER_TIME_OUT);
  }
}
