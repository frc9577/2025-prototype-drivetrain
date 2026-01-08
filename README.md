# 2025-prototype-drivetrain
FRC 9577's code repository for their fall 2025 custom drivetrain project.

## Repository Organization
This repository is organized to have 2 "primary" branches, these are non-temporary branches which are the basis of all other branches. 

They are the following:
<br>&nbsp;&nbsp;&nbsp;&nbsp;**main** - Code that is known to work via testing, does NOT have any features that are still in development or half done.
<br>&nbsp;&nbsp;&nbsp;&nbsp;**development** - Code that is not stable enough for main, either it has features that are not completed or is not tested.

There are also "feature" branches, these branches are temporary for the development of specific features. The feature is listed in the name of the branch, this allows for parallel development of different features that may require the same files. Once a feature is finished and is tested it can be PRed into the development branch and then deleted.

Main can only be PRed into and cannot have any direct commits, development is usually the branch that is PRed into main. All feature branches are PRed into development after the feature is completed for final testing and integration. 

The primary branches can be merged into the feature branches at any time to update config and readme files, to keep them the same across all branches (when it makes sense). When changing constants, please make those changes on development then cherry-pick or merge development into your branch to make sure there are no duplicated values.

## Overall Goals & Definitions
The goal of the project is to create a stable drivetrain that can drive smoothly with short cycle times.
We will NOT have a configurable chassis.

### Competitive Cycle Times
  - "Cycle Time" is defined as "The time from the start of game piece control to scoring that game piece"
  - Using "Competive" because the time changes depending on what game is being played.

### Stability
- The ability to go fast without tipping & wobbling
- Little to no bounciness
- Control stability

### Smooth Driving
- Don't stop to turn
- Follow a smooth path

## Requirements
The requirements set out for the robot.

- The robot shall comply with the 2025 FRC rules,
- Shall have 6 wheels,
- Shall use wheels 4 inches in diameter,
- Shall have center wheels powered by gearboxes,
- Shall have outer wheels powered by timing belts connected to center wheels,
- Shall have a maximum speed of 18 ft/s,
- Shall stay in an upright orientation,
- Shall have a low center of gravity,
- Shall have a center of gravity close to the center of the vertical projection of the robot,
- **Shall use acceleration profiles**,
- Shall have an IMU,
- Shall be constructed from 2”x1” ⅛” wall aluminum tubing,
