package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.PivotSubsystem;

public class SetPivotAngleCmd extends Command {
  private final PivotSubsystem pivotSubsystem;

  private final double angle;

  public SetPivotAngleCmd(PivotSubsystem pivotSubsystem, double angle) {
    this.pivotSubsystem = pivotSubsystem;
    this.angle = angle;

    addRequirements(pivotSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivotSubsystem.setPivotAngle(angle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
