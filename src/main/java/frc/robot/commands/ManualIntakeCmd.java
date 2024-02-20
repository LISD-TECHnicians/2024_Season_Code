package frc.robot.commands;

import java.util.function.BooleanSupplier;

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

  private final BooleanSupplier speed;

  public ManualIntakeCmd(IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, IndexerSubsystem indexerSubsystem,
      BooleanSupplier speed) {
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    this.speed = speed;

    addRequirements(intakeSubsystem, pivotSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(PivotConstants.INTAKE_ANGLE);

    if (pivotSubsystem.getIntakeReadiness()) {
      intakeSubsystem.setIntakeSpeed(speed.getAsBoolean() ? IntakeConstants.INTAKE_DEFAULT_SPEED : 0.0);
      indexerSubsystem.setIndexerSpeed(IndexerConstants.INDEXER_DEFAULT_SPEED);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return indexerSubsystem.getNotePresent();
  }
}
