package frc.robot.utils;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoChooser {
    private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

    private final HashMap<String, Command> autoCommands = new HashMap<>();

    public AutoChooser() {
        // load all autos before start of match
        for (String autoName : getAllAutoNames()) {
            //autoCommands.put(autoName, new PathPlannerAuto(autoName));
            m_autoChooser.addOption(autoName, autoName);
        }
        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    public String getSelectedAutoModeName() {
        //return m_autoChooser.getSelected();
        return "";
    }

    public Optional<Pose2d> getStartingAutoPose() {
        Optional<Pose2d> startingAutoPose = getSelectedAutoModeName() == "" ? Optional.empty()
                : Optional.of(PathPlannerAuto.getStaringPoseFromAutoFile(getSelectedAutoModeName()));
        return startingAutoPose;
    }

    public Command getSelectedAutoCommand() {
        if (autoCommands.containsKey(getSelectedAutoModeName())) {
            return autoCommands.get(getSelectedAutoModeName());
        }
        return new InstantCommand(); // empty command
    }

    private static List<String> getAllAutoNames() {
        File[] autoFiles = new File(Filesystem.getDeployDirectory(), "pathplanner/autos").listFiles();

        if (autoFiles == null) {
            return new ArrayList<>();
        }

        return Stream.of(autoFiles)
                .filter(file -> !file.isDirectory())
                .map(File::getName)
                .filter(name -> name.endsWith(".auto"))
                .map(name -> name.substring(0, name.lastIndexOf(".")))
                .collect(Collectors.toList());
    }
}
