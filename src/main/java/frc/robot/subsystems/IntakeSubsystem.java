package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intake = new CANSparkMax(IntakeConstants.INTAKE_ID, MotorType.kBrushless);

  public IntakeSubsystem() {
    intake.restoreFactoryDefaults();

    intake.enableVoltageCompensation(ControllerConstants.NOMINAL_VOLTAGE);

    intake.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);

    intake.burnFlash();
  }

  public void setIntakeSpeed(double speed) {
    intake.set(speed * IntakeConstants.INTAKE_SPEED_FACTOR);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
