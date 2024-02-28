package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeConstants;

import frc.robot.subsystems.IntakeSubsystem;

public class ManualReverseIntakeCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;

  public ManualReverseIntakeCmd(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    intakeSubsystem.setIntakeSpeed(-IntakeConstants.INTAKE_DEFAULT_SPEED);
  }

  @Override
  public void end(boolean interrforwardted) {
    intakeSubsystem.setIntakeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
