# How To

## vendordeps

If vendor imports aren't working (such as for REV robotics), try in the terminal

```
> .\gradlew vendordep --url=https://software-metadata.revrobotics.com/REVLib.json
```

## ssh into roboRio

```
> ssh admin@10.56.87.2
```

Logs on USB thumb drive are in /media/sda1

Copy log file from roboRio

```
> scp admin@10.56.87.2:/media/sda1/log_2022-03-02_173947.txt .
```

## view console errors in Driver Station

The problem is that if code panics, it automatically restarts. This makes it impossible to use consoler to see what the error is b/c it keeps scrolling off the screen and there is no way to pause. Solution is to jump off WiFi so console doesn't keep getting updated. Then you can see the errors in console without it scrolling on you.

## See current log on roboRio

The current session log is at

/var/local/natinst/log/FRC_UserProgram.log

To filter out all the navX-Sensor messages,

grep -Ev '(navX-Sensor)' FRC_UserProgram.log | more

To get just the info and debug messages,

grep -E '(info|debug)' FRC_UserProgram.log | more