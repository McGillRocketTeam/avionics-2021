# avionics-2020
Contains all projects in Avionics other than ground-station. 

_See ground-station repo [here](https://github.com/McGillRocketTeam/ground-station-2020)._

Please ASK if you have any questions.

## Branches
* master
* dev
* feature branch (optional)

## Branching rules

* The ___master___ branch will be downloaded for competition. This means that it will contain all final changes.
* The ___dev___ branch will be the working copy before all final versions get merged into "master". 
* All ___feature___ branches should branch off ___dev___. 

## Feature branch naming convention

* yourname/featureName

Eg. Branching off ___dev___ to create a feature branch
```
git checkout dev
git checkout -b mei/anechoicChamberTest
```

## Folder directory rules

Create folders for each project and put your project-specific files in the respective folders to avoid merge conflicts. Feel free to create any necessary subfolders to make everything cleaner.

Eg. 

```bash

Avionics
.
├── Ejection
|   └── Testing
├── Embedded R&D
|   └── Testing
├── Telemetry
|   ├── Antenna
|   ├── Radios
|   ├── Sensors
|   └── Testing
└── Video recorder
    └── Testing
            
```