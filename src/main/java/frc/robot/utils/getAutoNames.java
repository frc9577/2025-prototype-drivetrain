package frc.robot.utils;

import java.io.File;
import java.util.ArrayList;

// This goes to "/home/lvuser/deploy/pathplanner/autos" and grabs the names of all .auto files
// We are asuming that the PathPlannerAuto(string) is the file name of the auto
// We are asuming that the files are always in the "/home/lvuser/deploy/pathplanner/autos"
// We found this information at https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage
public class getAutoNames {
    public String[] main() {
        // Creating an ArrayList so I can fill it with auto file names w/o the extention.
        ArrayList<String> autoNames = new ArrayList<String>();

        // Getting the folder and files.
        File folder = new File("/home/lvuser/deploy/pathplanner/autos");
        File[] listOfFiles = folder.listFiles();

        // Checking if the files exist
        if (listOfFiles != null) {
            for (File file : listOfFiles) {
                if (file.isFile()) {
                    String fullName = file.getName(); // Returns [file_name].[extention]
                    String[] splitName = fullName.split("."); // Splitting the file name into seprate parts.
                    
                    // If its an auto file then add it to the list.
                    if (splitName[1] == "auto") {
                        autoNames.add(splitName[0]);
                        System.out.println(splitName[0]);
                    }
                }
            }
        }

        // Converting the ArrayList into a normal array for memory optimization & saftey
        String[] autoNamesArray = new String[autoNames.size()];
        autoNames.toArray(autoNamesArray);

        // Returning the new array
        return autoNamesArray;
    }
}
