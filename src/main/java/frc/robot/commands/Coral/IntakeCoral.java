package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ControllerVibrate;
import frc.robot.subsystems.Coral;

public class IntakeCoral extends Command{
    
    private final Coral coral;
    private final DigitalInput sensor;
    ControllerVibrate controllerVibrate;

    
    public IntakeCoral(Coral coral, DigitalInput sensor){
        this.coral = coral;
        this.sensor = sensor;
        controllerVibrate = new ControllerVibrate(1);
    }

    @Override
    public void initialize() {
        coral.moveCoral();
        System.out.println("IntakeCoral.initialize");
    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean isFinished){
        coral.stopCoral();
        controllerVibrate.schedule();
        System.out.println("IntakeCoral.end()");
    }

    @Override
    public boolean isFinished(){
        return !sensor.get();
    }

}