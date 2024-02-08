package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.DriveConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax climberLeft = new CANSparkMax(ClimberConstants.CLIMBER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax climberRight = new CANSparkMax(ClimberConstants.CLIMBER_RIGHT_ID, MotorType.kBrushless);
  
  private final SlewRateLimiter climberRateLimiter = new SlewRateLimiter(ClimberConstants.CLIMBER_RATE_LIMIT);

  public ClimberSubsystem() {
    climberLeft.enableVoltageCompensation(DriveConstants.NOMINAL_VOLTAGE);

    climberLeft.setIdleMode(IdleMode.kBrake);
    climberRight.setIdleMode(IdleMode.kBrake);

    climberLeft.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.CLIMBER_FORWARD_LIMIT);
    climberLeft.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.CLIMBER_REVERSE_LIMIT);

    climberLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
    climberLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);

    climberRight.follow(climberLeft, true);
  }

  public void setClimberSpeed(double speed) {
    climberLeft.set(climberRateLimiter.calculate(speed));
  }

  public boolean getLowerLimit() {
    return climberLeft.getEncoder().getPosition() == climberLeft.getSoftLimit(SoftLimitDirection.kReverse);
  }

  public boolean getUpperLimit() {
    return climberLeft.getEncoder().getPosition() == climberLeft.getSoftLimit(SoftLimitDirection.kForward);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
