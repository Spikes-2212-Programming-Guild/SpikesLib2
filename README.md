# SpikesLib2

SpikesLib is a library that extends [WPILib](https://github.com/wpilibsuite/allwpilib) written by the FRC team The Spikes#2212.


## 2020 Season

For this season, SpikesLib was moved to this repository due to the extensive changes in WPILib. <br>
The old version is still available at the [old repository](https://github.com/Spikes-2212-Programming-Guild/SpikesLib)


## Packages

- command - extensions for WPILib's Command Based framework
- control - controller wrappers and custom controllers
- path - path following code and pure pursuit algorithm
- util - additional utilities


## Installation
After creating a Robot Project and importing the CTRE libraries, change the build.gradle as written:<br><br>

1. Add: <br>
   ```
   repositories {
       maven { 
           url "https://jitpack.io"
       }
    }
   ```
   <br>
2. Add: <br>
   ```
   dependencies {
       implementation 'com.github.Spikes-2212-Programming-Guild:SpikesLib2:master'
   } 
   ```
Then, reload gradle and perform a build.

## Development

SpikesLib is developed in a feature branches workflow. <br>
All feature branches are merged into _```dev```_ branch after testing, which in turn is merged into 
_```master```_ after it passing complete testing <br>

Branches should be named according to the following convention - _```name-package-feature```_
