package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax climberLeft = new CANSparkMax(ClimberConstants.CLIMBER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax climberRight = new CANSparkMax(ClimberConstants.CLIMBER_RIGHT_ID, MotorType.kBrushless);

  public ClimberSubsystem() {
    // Enable Soft Limits

    climberRight.follow(climberLeft, true);
  }

  public void setClimberSpeed(double speed) {
    climberLeft.set(speed);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
