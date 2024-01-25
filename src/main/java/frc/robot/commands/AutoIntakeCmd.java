package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AutoIntakeCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  // Swerve susbystem to reverse slowly

  private final boolean autoAlign;

  public AutoIntakeCmd(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, LimelightSubsystem limelightSubsystem, boolean autoAlign) {
    this.intakeSubsystem = intakeSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    this.autoAlign = autoAlign;

    addRequirements(intakeSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (autoAlign) {
      if (limelightSubsystem.getTX(LimelightConstants.LL_ONE) < 2) {
        intakeSubsystem.setIntakeSpeed(1);
        indexerSubsystem.setIndexerSpeed(0.5);
        // move back as quick as is reliable
      }
      else {
        // rotate until get TX is low
      }

    }
    else {
      intakeSubsystem.setIntakeSpeed(1);
      indexerSubsystem.setIndexerSpeed(0.5);
      // move back as quick as is reliable
    }
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeSpeed(0);
    indexerSubsystem.setIndexerSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return indexerSubsystem.getNotePresent();
  }
}
