package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  private final DoubleSupplier speed;

  public RunIntakeCmd(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, DoubleSupplier speed) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    this.speed = speed;

    addRequirements(intakeSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.setIntakeSpeed(speed.getAsDouble());
    indexerSubsystem.setIndexerSpeed(0.5);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return indexerSubsystem.getNotePresent();
  }
}
