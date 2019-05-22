## AirSim Setup

This is a temporary page describing the development setup of AirSim and how to use Ardupilot SITL with it (currently under development)

### Linux

This AirSim page describes how to build Unreal Engine, AirSim
<https://microsoft.github.io/AirSim/docs/build_linux/>

Setting up the Blocks Environment - <https://microsoft.github.io/AirSim/docs/unreal_blocks/>

Development Workflow in AirSim -  

- Updating the repo - Normal Git workflow
- Make any changes required
- Run the `build.sh` script in Airsim which will also copy the changes to the plugin directory
- Launch UnrealEngine Editor, choose Blocks environment and when prompted about missing .so files, press Yes to build it again.

- Troubleshooting - <https://microsoft.github.io/AirSim/docs/build_linux/>

### Windows

Build AirSim on Windows - <https://microsoft.github.io/AirSim/docs/build_windows/>

Setup Blocks Environment - <https://microsoft.github.io/AirSim/docs/unreal_blocks/>

Development Workflow - <https://microsoft.github.io/AirSim/docs/dev_workflow/>

### Using ArduCopterSolo vehicle

See <https://github.com/Microsoft/AirSim/blob/master/docs/settings.md> for info about settings

Current `settings.json` file for launching Solo

```
{
  "SettingsVersion": 1.2,
  "LogMessagesVisible": true,
  "SimMode": "Multirotor",
  "OriginGeopoint": {
    "Latitude": -35.363261,
    "Longitude": 149.165230,
    "Altitude": 583
  },
  "Vehicles": {
      "Solo": {
          "VehicleType": "arducoptersolo",
          "UseSerial": false,
          "AllowAPIAlways": false,
          "UdpIp": "127.0.0.1",
          "UdpPort": 14560,
          "SitlIp": "127.0.0.1",
          "SitlPort": 14556 
        }
    }
}
```

Note: Many of the fields have the default values as above only but just specifying them

First launch AirSim, after that launch the ArduPilot SITL using 
```
sim_vehicle.py -v ArduCopter -f airsim --console --map
```

Current state - Everything's connected, Gyro initialization is successful after using EKF type 10 but it's setting random GPS location

You'll probably need to force close AirSim till everything's working properly