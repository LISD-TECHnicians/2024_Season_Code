package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ControllerConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberLeft = new TalonFX(ClimberConstants.CLIMBER_LEFT_ID);
  private final TalonFX climberRight = new TalonFX(ClimberConstants.CLIMBER_RIGHT_ID);

  public ClimberSubsystem() {
    climberLeft.getConfigurator().apply(new TalonFXConfiguration());
    climberRight.getConfigurator().apply(new TalonFXConfiguration());

    climberLeft.setNeutralMode(NeutralModeValue.Brake);
    climberRight.setNeutralMode(NeutralModeValue.Brake);

    climberRight.setControl(new Follower(climberLeft.getDeviceID(), true));
  }
  
  public void setClimberSpeed(double speed) {
    if (speed > 0 && getUpperLimit()) {
      climberLeft.setVoltage(0); 
    }
    else if (speed < 0 && getLowerLimit()) {
      climberLeft.setVoltage(0);
    }
    else {
      climberLeft.setVoltage(speed * ControllerConstants.NOMINAL_VOLTAGE);
    }
  }

  public double getClimberPosition() {
    return climberLeft.getPosition().getValueAsDouble();
  }

  public boolean getUpperLimit() {
    return Math.abs(getClimberPosition() - ClimberConstants.CLIMBER_FORWARD_LIMIT) < ClimberConstants.CLIMBER_VARIABILITY;
  }

  public boolean getLowerLimit() {
    return Math.abs(getClimberPosition() - ClimberConstants.CLIMBER_REVERSE_LIMIT) < ClimberConstants.CLIMBER_VARIABILITY;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
