package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;

  private final DoubleSupplier speed;

  public RunIntakeCmd(IntakeSubsystem intakeSubsystem, DoubleSupplier speed) {
    this.intakeSubsystem = intakeSubsystem;

    this.speed = speed;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.setIntakeSpeed(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
