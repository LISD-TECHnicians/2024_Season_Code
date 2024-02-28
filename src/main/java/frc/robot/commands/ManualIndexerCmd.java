package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IndexerConstants;

import frc.robot.subsystems.IndexerSubsystem;

import java.util.function.BooleanSupplier;

public class ManualIndexerCmd extends Command {
  private final IndexerSubsystem indexerSubsystem;

  private final BooleanSupplier forward;
  private final BooleanSupplier reverse;

  public ManualIndexerCmd(IndexerSubsystem indexerSubsystem, BooleanSupplier forward, BooleanSupplier reverse) {
    this.indexerSubsystem = indexerSubsystem;

    this.forward = forward;
    this.reverse = reverse;

    addRequirements(indexerSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (forward.getAsBoolean()) {
      indexerSubsystem.setIndexerSpeed(IndexerConstants.INDEXER_DEFAULT_SPEED);
    }
    else if (reverse.getAsBoolean()) {
      indexerSubsystem.setIndexerSpeed(-IndexerConstants.INDEXER_DEFAULT_SPEED);
    }
    else {
      indexerSubsystem.setIndexerSpeed(0);
    }
  }

  @Override
  public void end(boolean interrforwardted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
