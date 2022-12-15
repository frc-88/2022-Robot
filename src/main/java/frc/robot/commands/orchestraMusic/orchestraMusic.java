package frc.robot.commands.orchestraMusic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.drive.*;
import frc.robot.util.drive.DriveUtils;

import com.ctre.phoenix.music.Orchestra;

import java.lang.reflect.Array;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class orchestraMusic extends CommandBase {
    
    Orchestra m_orchestra;
    //puts all of our falcons in a list
    WPI_TalonFX falconMotorFxs[] = {
    new WPI_TalonFX(1, "FRONT_LEFT_DRIVE"), 
    new WPI_TalonFX(0, "FRONT_LEFT_STEER"),
    new WPI_TalonFX(18, "FRONT_RIGHT_DRIVE"), 
    new WPI_TalonFX(19, "FRONT_RIGHT_STEER"), 
    new WPI_TalonFX(8, "BACK_LEFT_DRIVE"),
    new WPI_TalonFX(9, "BACK_LEFT_STEER"), 
    new WPI_TalonFX(10, "BACK_RIGHT_DRIVE"), 
    new WPI_TalonFX(11, "BACK_RIGHT_STEER") 
    };
    
    String[] songs = new String[] {
        //where songs will go when created
    };

    int songSelection = 0;
    int timeToPlayLoops = 0;

    void LoadMusicSelection(int offset)
    {
        /* increment song selection */
        songSelection += offset;
        /* wrap song index in case it exceeds boundary */
        if (songSelection >= songs.length) {
            songSelection = 0;
        }
        if (songSelection < 0) {
            songSelection = songs.length - 1;
        }
        /* load the chirp file */
        m_orchestra.loadMusic(songs[songSelection]); 

    }
    @Override
    public void initialize() {
        ArrayList<TalonFX> motorInstruments = new ArrayList<TalonFX>();

        //loop adds each device in our list of falcon motors to
        //the orchestra object array
        for (int i = 0; i < falconMotorFxs.length; ++i) {
            motorInstruments.add(falconMotorFxs[i]);
        }

        m_orchestra = new Orchestra(motorInstruments);
        
    }

    @Override
    public void execute() {
        LoadMusicSelection(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
