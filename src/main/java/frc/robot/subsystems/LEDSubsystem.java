package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LEDConstants;

import java.awt.*;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle candle = new CANdle(LEDConstants.CANDLE_PORT);

    public LEDSubsystem() {}

    public void setColor(Color color) {
        setColorAndBrightness(color, 1);
    }

    public void setColorAndBrightness(Color color, double brightness) {
        this.candle.configBrightnessScalar(brightness);

        this.candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue());
    }
}
