package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle LED = new CANdle(LEDConstants.CANDLE_ID, "canivore"); 
  
  public LEDSubsystem() {
    LED.configFactoryDefault();

    LED.configLEDType(LEDStripType.GRB);
    LED.configBrightnessScalar(LEDConstants.CANDLE_BRIGHTNESS);
  }

  public void setColor(int r, int g, int b) {
    LED.setLEDs(r, g, b, 0, 0, 90);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
