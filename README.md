# 2025_r
Robot Code &amp; Resources for PantherTech's 2025 season.

# Resources

### Things we might want to try this season & Goals

- Elastic Dashboard - https://www.chiefdelphi.com/t/elastic-2025-a-lot-of-little-things/472475
- Motion Magic for mechanism position control - https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/motion-magic.html
   - https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/MotionMagic
- Robot Alerts - Either elastic or the new ones in WPILib
- Logging
  - CTRE Signal Logging: https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/signal-logging.html
  - AdvantageScope: https://docs.advantagescope.org/
  - WPI Logs: https://docs.wpilib.org/en/stable/docs/software/telemetry/robot-telemetry-with-annotations.html

- SysID: https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
- Use new WPILib LED Code (No more coding lights for hours!) - https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html#led-patterns
- Object Detection? (LimeLight 4)
- Perhaps try Choreo? It factors in robot properites like weight, so you can get the most optiomal speeds. Also can directly supply paths to PathPlannerLib, no code changes needed.
  - https://docs.wpilib.org/en/stable/docs/software/pathplanning/choreo/index.html
  - https://www.chiefdelphi.com/t/choreo-vs-pathplanner/467373
- GitHub Actions

### Things to different this year
- Make sure to tune Drive PID!
- Make sure to add code to invert controls if RED alliance.

### General Resources
- Phoenix 6 Swerve with Limelight - https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/7b0ad9e716ac6a1462f225d86c89842defacdaa2/java/SwerveWithPathPlanner/src/main/java/frc/robot/Robot.java#L29-L47
