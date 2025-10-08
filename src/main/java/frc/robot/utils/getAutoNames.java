package frc.robot.utils;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

// This goes to "/home/lvuser/deploy/pathplanner/autos" and grabs the names of all .auto files
// We are asuming that the PathPlannerAuto(string) is the file name of the auto
// We are asuming that the files are always in the "/home/lvuser/deploy/pathplanner/autos"
// We found this information at [URL HERE]
public class getAutoNames {
    public List<String> main() {
        File directory = new File("/home/lvuser/deploy/pathplanner/autos");

        List<String> fileNames = new ArrayList<>();
        return fileNames;
    }
}
