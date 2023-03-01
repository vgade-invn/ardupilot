Vision Aerial advance Continue Mission notes:

PARAMETERS NOTES

- GUID_OPTIONS must be set to 64: In order to use the newer waypoint navigation controller, which we are using on this feature

- ADFS_ENA manages if this procedure is enabled or not. 0 will have standard behaviour, 1 our custom one. It needs to be set to 1
  for this feature to work

- ADFS_ALT_CM: This is the altitude the Copter will use ( above home ) to return to mission. It will ascent ( or descent ) to this altitude,
  then go to the coordinate of the returning point, and then descent or ascent until it reaches the saved ASL altitude. Then it will set camera
  trigger state, and continue mission normally.

- ADFS_MAXDST_M: If return to mission point is farther away than ADFS_MAXDST_M (meters) the return to mission command will be rejected. Flight deck will 
  show the popup of mission command rejected, and if we click in the dialog box we will see extended info, what is the maximum limit, and the 
  actual distance the returning point is from the current location. When this happens, the vehicle will be left in guided mode, hovering, for security.
  The user will be able to change flight mode and continue normally.

GENERAL NOTES

- When the copter is returning to mission, even though it is in guided mode, it will not respond to goto commands (click on map). 
  in order to do so, we must change the flight mode to something else ( for example break ), and then click on map for goto ( click on map to go )
  feature to work.

- We have logging. ADFS group of values. TimeUS,lat,lon,alt,vel,tdi. Latest one is trigger distance, rest of them are self explanatory.

- When the copter is returning to the continue mission point, it is possible to stop ( with break or loiter mode ) in case we need to avoid
  any obstacle on our way to the mission. After the obstacle is avoided, we can toggle again continue mission slider to resume the procedure.

- Mission state will be saved anytime we get out of auto mode, and if we are at least past the first waypoint of mission.

- It will save ASL altitude. It will try to get asl altitude when going out of auto. If it doesn't succeed for some reason, it will 
  throw a warning message "Failed to save mission state, unable to get ASL altitude" and will not save the mission state.

- MAKE SURE ADFS_NW_* PARAMS ARE WRITE ONLY: This is what AP uses to save the state of the mission so it persists reboots. These should not
  be changeable by the user. Double check this is working correctly, and the user is indeed not able to overwrite this.
