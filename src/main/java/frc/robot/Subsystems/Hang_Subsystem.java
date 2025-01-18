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
    
    
    
    
    
    public void PneumaticsSubsystem() {
        final Solenoid o_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
        final Solenoid t_solenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);

        
    }

    public double initializeCompressor() {
        final Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableAnalog(70, 120);
        return compressor.getPressure();

    }
       
         

    

        
    
}
