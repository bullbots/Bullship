package frc.robot.commands;

import com.fasterxml.jackson.databind.ser.std.MapProperty;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandTemplate extends Command{
    


    
    public CommandTemplate(){
    }

    @Override
    public void initialize() {
    } 
    @Override
    public void execute(){

    }

    @Override
    public void end(boolean isFinished){
    }
    @Override
    public boolean isFinished(){
        return true;
    }

}
