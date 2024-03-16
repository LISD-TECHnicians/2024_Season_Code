package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ShooterConstants;

import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;

  public SpinUpCmd(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    shooterSubsystem.setShooterSpeed(ShooterConstants.SHOOTER_DEFAULT_SPEED);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setShooterSpeed(0.0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
