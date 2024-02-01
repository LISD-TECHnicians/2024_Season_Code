package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShootCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  private final DoubleSupplier ShooterSpeed;
  private final DoubleSupplier PivotAngle;
  private final DoubleSupplier IndexerSpeed;

  public ManualShootCmd(ShooterSubsystem shooterSubsystem, PivotSubsystem pivotSubsystem, IndexerSubsystem indexerSubsystem, 
      DoubleSupplier ShooterSpeed, DoubleSupplier PivotAngle, DoubleSupplier IndexerSpeed) {
    this.shooterSubsystem = shooterSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    this.ShooterSpeed = ShooterSpeed;
    this.PivotAngle = PivotAngle;
    this.IndexerSpeed = IndexerSpeed;

    addRequirements(shooterSubsystem, pivotSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(PivotAngle.getAsDouble());
    shooterSubsystem.setShooterSpeed(ShooterSpeed.getAsDouble());

    if (Math.abs(PivotAngle.getAsDouble() - pivotSubsystem.getPivotAngle()) < 1.0) {
      indexerSubsystem.setIndexerSpeed(IndexerSpeed.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
