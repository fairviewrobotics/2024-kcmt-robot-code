package frc.robot.subsystems;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle candle = new CANdle(61, "rio");

    private final int LedCount = 160;

    private int r = 0;
    private int g = 0;
    private int b = 0;

    private AnimationTypes currentAnimation = null;

    private Animation animation = null;

    private boolean pureRGB = false;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        GreenStrobe,
        RedStrobe,
        Twinkle,
        TwinkleOff,
        PureRGB,
        Off
    }

    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = CANdle.LEDStripType.RGB;
        config.brightnessScalar = 0.1;
    }

    public void setRGB(int r, int g, int b) {
        this.pureRGB = true;
        this.r = r;
        this.g = g;
        this.b = b;
//        this.candle.setLEDs(r, g, b);

    }

    public void setAnimation(AnimationTypes toChange) {
        currentAnimation = toChange;
        this.pureRGB = false;

        switch(toChange)
        {
            case ColorFlow:
                animation = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
                break;
            case Fire:
                animation = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                break;
            case Larson:
                animation = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
                break;
            case Rainbow:
                animation = new RainbowAnimation(1, 0.5, LedCount);
                break;
            case RgbFade:
                animation = new RgbFadeAnimation(0.7, 0.4, LedCount);
                break;
            case SingleFade:
                animation = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                break;
            case GreenStrobe:
                animation = new StrobeAnimation(0, 255, 0, 0, 0.1, LedCount);
                break;
            case RedStrobe:
                animation = new StrobeAnimation(255, 0, 0, 0, 0.1, LedCount);
                break;
            case Twinkle:
                animation = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                animation = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
                break;
            case Off:
                animation = null;
                r = 0;
                g = 0;
                b = 0;
                break;
        }
    }

    @Override
    public void periodic() {
        candle.animate(animation);
        if (this.pureRGB) candle.setLEDs(this.r, this.g , this.b);

//            pureRGB = false;

        //else if (pureRGB) {
//            candle.setLEDs(r, g, b);
////                pureRGB = true;
//        }

    }
}
