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