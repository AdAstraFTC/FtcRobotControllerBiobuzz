# Team31192 OpMode Guide

Welcome to your team's code module!

This is where **your** robot code lives. Everything you write or copy should stay inside this `Team31192` folder. The `FtcRobotController` module (the official FTC samples) should **never** be edited — it gets replaced when the FTC SDK updates.

## Quick Start – Copying a Sample OpMode

The easiest way to create your own OpMode is to copy a Sample and make it your own.

**Where to find samples**  
`FtcRobotController > java > org.firstinspires.ftc.robotcontroller > external > samples`

### Sample Naming System

| Prefix    | What it shows                                      | Best used for                     |
|-----------|----------------------------------------------------|-----------------------------------|
| **Basic**   | Smallest working OpMode (structure only)           | Your very first OpMode            |
| **Sensor**  | How to read one specific sensor                    | Testing new hardware              |
| **Robot**   | Simple drivetrain + one main idea                  | Basic driver-controlled robot     |
| **Concept** | One focused advanced topic                         | Vision, servos, PID, etc.         |

**Steps to copy a sample into your team code:**

1. In Android Studio, switch to **Project** view.
2. Expand: `FtcRobotController > java > ... > external > samples`
3. Right-click the sample `.java` file you want → **Copy**.
4. Expand your module: `Team31192 > java > org.firstinspires.ftc.teamcode`
5. Right-click the `teamcode` folder → **Paste**.
6. Give it a clear name (e.g. `TeleOp_TankDrive`, `Auto_Park`, `Test_ColorSensor`).
7. Update the `@TeleOp` line and remove `@Disabled` (see below).
8. If you see red errors: **Build > Clean Project**, then **Build > Rebuild Project**.

## Making Your OpMode Appear on the Driver Station

Near the top of every OpMode you will see lines like this:

```java
@TeleOp(name="Template: Linear OpMode", group="Linear OpMode")
@Disabled
```

**What to change:**
- Change the `name="..."` to something clear for your drivers.
- Set `group="TeleOp"`, `"Autonomous"`, or `"Test"`.
- **Delete or comment out** the `@Disabled` line.

Once you save and rebuild, your OpMode will appear on the Driver Station.

## Golden Rules (Memorize These)

- Your code belongs only in the `Team31192` module. Never edit anything inside `FtcRobotController`.
- Commit to Git after every successful change or test.
- Document your work: After you get an OpMode working, take a screenshot of the project tree and write one sentence in your notebook about what you learned or what you want to improve next.
- Ask good questions: When you need help, tell us what you already tried and ask a specific question.
- Your code must be your own work. You may study examples (including Coach’s code in `Team00000`), but you must understand and write your own.

## Quick Reference Checklist

- [ ] Choose the right sample using the table above
- [ ] Copy → Paste → Rename with a clear name
- [ ] Update `@TeleOp(name=..., group=...)` and remove `@Disabled`
- [ ] Clean & Rebuild if there are red errors
- [ ] Test on the Driver Station
- [ ] Screenshot + one notebook sentence (Golden Rules)

---

Start simple. Copy one **Basic** sample, get it running on the Driver Station, and document it. Then build from there.

You’ve got this. Let’s build something great.

— Coach Chris & the Programming Mentors