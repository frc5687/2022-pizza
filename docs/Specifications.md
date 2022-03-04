# Functional Specifications

Why do we write out our functional specifications? So no one says two days before the first match, "No one told me the robot had to be able to do *that*!"

## Will Do / Can Do Table

As of February 25, 2022.

Category | Specification | Will Do | Pizza Can Do | Comp Can Do
---------|---------------|---------|----------------|------------
Move | Move responding to remote human manual control | YES | YES | YES
Move | Translate and rotate at the same time | YES | YES | YES
Shoot | Shoot cargo into high goal | YES | NO | YES
Shoot | Shoot and Move | NO | NO | NO
Shoot | Check color using color sensor | YES | NO | NO 
Shoot | Eject opponent cargo at low power| YES | NO | NO
Auto | Complete an autonomous routine | YES | YES | YES
Auto | Zero Ball Auto from p3  | YES | NO | NO
Auto | Zero Ball Auto, from p1, p2, p3, p4 | YES | NO | NO
Auto | One Ball Auto, from p1, p2, p3, p4 | YES | NO | NO
Auto | Two Ball Auto, from p1, p2, p4 | YES | NO | NO
Auto | Three Ball Auto, from p1, p2 | YES | NO | NO
Auto | Four Ball Auto, from p1 | YES | NO | NO
Auto | Five Ball Auto, from p1 | YES | NO | NO
Climb | Climb to Top Bar | YES | NO | NO
V-SLAM | Localize Ourself on the field | YES | NO | NO
Fly | Levitate in place | NO | NO | NO 

# Categories

  ## Move
  
  - Translate left, right, up, down.
  - Rotate.
    
  ## Shoot
  Ideal shoot anywhere on field.
  
  - Check if arm is lowered & if not lowered, lower arm.
  - Check if cargo is loaded & check cargo color.
  - If empty, load cargo & check cargo color.
    
  - If wrong cargo color, set springs to low strength and eject cargo.
    
  - Look for target using vision system.
  - Center shooter with target.
  - Determine distance to target.
  - Adjust spring tension to achieve correct parabola.
  - Release pin to fire cargo.
  - Pull arm down for reloading.
    
  ## Intake
  
  - Put intake out / release intake.
  - Intake balls by rotating rollers.
  - Pull intake in / retract intake.  
  ## Display Information
  - Climbing = Fun pattern.  
  - Shooting fail = red flashing (we didn't shoot the ball).
  - Shooting success = Yellow flashing light(we shot the ball).
  - Intake = purple patterns. 
  - If centered with target = green flashing.
  
  ## Climb
  Start climbing at 45 seconds or more.
  
  - Drive to hangar.
  - Set up under mid bar.
  - Raise left arm.
  - Raise off of ground.
  - Raise right arm and grab high bar.
  - Pivot center of mass to under high bar.
  - Release left arm and retract.
  - Raise robot higher.
  - Pivot left arm for traversal rung.
  - Extend left arm.
  - Grab traversal rung with left arm.
  - Pivot center of mass under traversal rung.
  - Release right arm.
  