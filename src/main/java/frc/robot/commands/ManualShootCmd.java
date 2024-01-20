package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShootCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  private final DoubleSupplier ShooterSpeed;
  private final DoubleSupplier PivotAngle;
  private final DoubleSupplier IndexerSpeed;

  public ManualShootCmd(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, 
      DoubleSupplier ShooterSpeed, DoubleSupplier PivotAngle, DoubleSupplier IndexerSpeed) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;

    this.ShooterSpeed = ShooterSpeed;
    this.PivotAngle = PivotAngle;
    this.IndexerSpeed = IndexerSpeed;

    addRequirements(shooterSubsystem, indexerSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooterSubsystem.setPivotAngle(PivotAngle.getAsDouble());
    shooterSubsystem.setShooterSpeed(ShooterSpeed.getAsDouble());

    if (Math.abs(PivotAngle.getAsDouble() - shooterSubsystem.getPivotAngle()) < 1.0) {
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
