package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax climberLeft = new CANSparkMax(ClimberConstants.CLIMBER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax climberRight = new CANSparkMax(ClimberConstants.CLIMBER_RIGHT_ID, MotorType.kBrushless);

  private final DigitalInput LowerLimit = new DigitalInput(ClimberConstants.LOWER_LIMIT_PORT);

  public ClimberSubsystem() {
    climberLeft.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    climberLeft.setSoftLimit(SoftLimitDirection.kForward, 0);
    climberLeft.setSoftLimit(SoftLimitDirection.kReverse, 0);

    climberLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
    climberLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);

    climberRight.follow(climberLeft, true);
  }

  public void setClimberSpeed(double speed) {
    if (LowerLimit.get() == true && speed < 0) {
      climberLeft.set(0);
    }
    else {
      climberLeft.set(speed);
    }
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
