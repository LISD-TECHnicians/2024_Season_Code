package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimberSubsystem;

public class ManualClimberCmd extends Command {
  private final ClimberSubsystem climberSubsystem;

  private final DoubleSupplier speed;

  public ManualClimberCmd(ClimberSubsystem climberSubsystem, DoubleSupplier speed) {
    this.climberSubsystem = climberSubsystem;

    this.speed = speed;

    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    climberSubsystem.setClimberSpeed(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
