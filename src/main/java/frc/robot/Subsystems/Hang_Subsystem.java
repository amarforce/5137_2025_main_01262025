package frc.robot.Subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Hang_Constants.Pneumatics;
import frc.robot.Constants.Hang_Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.hal.REVPHJNI;
import edu.wpi.first.wpilibj.Compressor;




public class Hang_Subsystem extends SubsystemBase {
    
    private final Solenoid clamp_solenoid;
    private final Solenoid climb_solenoid;

    
    
    
    public void PneumaticsSubsystem() {
        final Solenoid clamp_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
        final Solenoid climb_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);

        
    }

    public double initializeCompressor() {
        final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableAnalog(70, 120);
        return compressor.getPressure();

        
    }

    public void clampActivate(){
        clamp_solenoid.set(true);

    }
    public void clampDeactivate(){
        clamp_solenoid.set(false);

    }
    public boolean isClampActivated(){
        return clamp_solenoid.get();

    }
    public void climbExtend(){
        climb_solenoid.set(true);

    }
    public void climbRetract(){
        climb_solenoid.set(false);

    }
    public void isclimbExtended(){
        climb_solenoid.get();

    }
       
         

    

        
    
}
