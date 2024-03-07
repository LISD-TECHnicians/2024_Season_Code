package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ShooterConstants;

import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;

  private final BooleanSupplier spinUp;

  public SpinUpCmd(ShooterSubsystem shooterSubsystem, BooleanSupplier spinUp) {
    this.shooterSubsystem = shooterSubsystem;

    this.spinUp = spinUp;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooterSubsystem.setShooterSpeed(spinUp.getAsBoolean() ? ShooterConstants.SHOOTER_DEFAULT_SPEED : 0);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterSpeed(1.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
