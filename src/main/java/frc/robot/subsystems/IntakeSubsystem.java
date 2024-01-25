package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeTop = new CANSparkMax(IntakeConstants.INTAKE_TOP_ID, MotorType.kBrushless);
  private final CANSparkMax intakeBottom = new CANSparkMax(IntakeConstants.INTAKE_BOTTOM_ID, MotorType.kBrushless);

  public IntakeSubsystem() {
    intakeTop.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    intakeBottom.follow(intakeTop, true);
  }

  public void setIntakeSpeed(double speed) {
    intakeTop.set(speed);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
