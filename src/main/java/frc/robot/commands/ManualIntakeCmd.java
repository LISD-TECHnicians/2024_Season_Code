package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ManualIntakeCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  private final Timer intakeTimeOut = new Timer();

  public ManualIntakeCmd(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, IndexerSubsystem indexerSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    addRequirements(intakeSubsystem, pivotSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {
    intakeTimeOut.restart();
  }

  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(PivotConstants.INTAKE_ANGLE);

    if (pivotSubsystem.getIntakeReadiness()) {
      intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_DEFAULT_SPEED);
      indexerSubsystem.setIndexerSpeed(IndexerConstants.INDEXER_DEFAULT_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeSpeed(0);
    indexerSubsystem.setIndexerSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return !indexerSubsystem.getNotePresent() || intakeTimeOut.hasElapsed(5);
  }
}
