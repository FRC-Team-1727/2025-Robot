package frc.robot.subsystems;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Mode;
import frc.robot.RobotContainer;

public class LEDSubsystem extends SubsystemBase{
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDSim sim;
    private LEDPattern pattern;
    private LEDMode mode;
    private int animStart;

    public LEDSubsystem(){
        mode = LEDMode.kDefault;
        pattern = LEDPattern.solid(Color.kRed);
        led = new AddressableLED(1);
        buffer = new AddressableLEDBuffer(24);
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();

        animStart = 0;
        sim = AddressableLEDSim.createForChannel(0);
        pattern = LEDPattern.rainbow(255, 128);
    }
    public void setMode(LEDMode mode){
        if(!(this.mode == mode)){
            this.mode = mode;
        }
        animStart = 0;
    }
    public void periodic(){
        if(RobotContainer.getMode() == Mode.CORALMODE){
            setMode(LEDMode.kCoralMode);
        }else{
            setMode(LEDMode.kAlgaeMode);
        }
        if(mode == LEDMode.kEmpty){
            setColor(Color.kRed);
        }else if(mode == LEDMode.kPartyTime){
            oldPartyMode();
        }else if(mode == LEDMode.kCoralMode){
           //setColor(Color.kCoral);
        //    randomColor(Color.kRed,Color.kBlue);
            PARTYMODE();
        }else if(mode == LEDMode.kAlgaeMode){
            setColor(Color.kAquamarine);
        }else if(mode == LEDMode.kAligning){
            aligning();
        }
        else if(mode == LEDMode.kAligned){
            aligned();
        }
        led.setData(buffer);
    }
    public void setColor(Color color){
        pattern = LEDPattern.solid(color);
        pattern.applyTo(buffer);
        // for(int i = 0; i < buffer.getLength();i++)
        // {
        //     buffer.setLED(i , color);

        // }
    }
    public void breatheColor(Color color){
        LEDPattern base = LEDPattern.solid(color);
        pattern = base.breathe(Seconds.of(2));
        pattern.applyTo(buffer);
    }

    public void randomColor(Color color, Color color2){
        pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,color,color2);
        pattern.scrollAtAbsoluteSpeed(InchesPerSecond.of(1), Meters.of(1/60));
        pattern.applyTo(buffer);
    }
    public void PARTYMODE(){
        
        pattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1 / 120.0));
        pattern.applyTo(buffer);
    }
    public void oldPartyMode(){
        for(int i = 0; i < buffer.getLength(); i++){
            int hue = animStart + (i * 180 / buffer.getLength()) % 180;
            buffer.setHSV(i, hue, 255,255);
        }
        animStart += 3;
        animStart %= 180;
    }
    public void aligning(){
        breatheColor(Color.kRed);
    }
    public void aligned(){
        setColor(Color.kGreen);
    }
}
