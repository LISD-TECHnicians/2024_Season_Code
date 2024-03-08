package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.PivotConstants;

import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.Drive.SwerveSubsystem;

public class AutoIntakeCmd extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final LimelightSubsystem limelightSubsystem;

  private final boolean autoAlign;

  public AutoIntakeCmd(SwerveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, 
      IndexerSubsystem indexerSubsystem, LimelightSubsystem limelightSubsystem, boolean autoAlign) {
    this.swerveSubsystem = swerveSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    this.autoAlign = autoAlign;

    addRequirements(swerveSubsystem, intakeSubsystem, pivotSubsystem, indexerSubsystem, limelightSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(PivotConstants.INTAKE_ANGLE);
    
    if (autoAlign) {
      swerveSubsystem.setSwerveRotation(limelightSubsystem.getTX(LimelightConstants.LL_ONE), DriveConstants.SWERVE_INTAKE_OFFSET);
      
      if (swerveSubsystem.getRotationReadiness(limelightSubsystem.getTX(LimelightConstants.LL_ONE), DriveConstants.SWERVE_INTAKE_OFFSET) 
          && pivotSubsystem.getIntakeReadiness()) {
        swerveSubsystem.setChassisSpeeds(DriveConstants.INTAKE_REVERSE_SPEED);
        intakeSubsystem.setIntakeSpeed(IntakeConstants.INTAKE_DEFAULT_SPEED);
        indexerSubsystem.setIndexerSpeed(IndexerConstants.INDEXER_DEFAULT_SPEED);
      }

    }
    else if (pivotSubsystem.getIntakeReadiness()) {
      swerveSubsystem.setChassisSpeeds(DriveConstants.INTAKE_REVERSE_SPEED);
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
    return !indexerSubsystem.getNotePresent();
  }
}
