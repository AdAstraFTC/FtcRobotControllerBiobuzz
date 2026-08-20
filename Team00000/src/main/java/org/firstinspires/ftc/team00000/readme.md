# TeamCode Guide for Students

**From Coach Chris Lemoine**  
FTC Lead Coach & Programming Mentor  
Starbase, Texas

Hello everyone,

This guide is written to **all three of our FTC teams** this season. We are using a varsity / JV / sub-varsity structure so programmers in grades 6–12 can develop at the right pace while still working together.

- **Primary Team (Varsity)** – Team 31192. Lead Android Studio team.
- **Secondary Team (JV)** – Building fundamentals and moving into Android Studio.
- **Third Tier Team (Sub-Varsity)** – Learning core concepts and gaining confidence before advancing.

**Our shared goal** is strong programming skills with Android Studio as the long-term environment. Mentors will meet every team at its current level while using this guide as the roadmap.

This guide has been streamlined (v2) so you can find what you need quickly without re-reading the same instructions multiple times. My working code lives in the `Team00000` module right alongside yours. Study the patterns — do not copy the files.

---

## 1. Where Your Code Lives

In our project there are two modules:

- **FtcRobotController** — Official FTC software and samples. **Never edit anything inside this module.**
- **Your Team Module** (`Team00000` or your numbered folder) — This is where **your** code lives. Everything you write stays here.

When the FTC SDK updates, `FtcRobotController` is replaced. Your work is safe only if it stays in your own module.

**Portfolio Prompt #1**  
In your engineering notebook, draw a simple diagram showing the two modules. Label which one belongs to you. Write one sentence explaining why we keep our code separate.

---

## 2. How Sample OpModes Are Organized

All samples live in:  
`FtcRobotController > java > org.firstinspires.ftc.robotcontroller > external > samples`

### Sample Naming System (Learn This)

| Prefix   | What It Shows                          | Best Time to Use                  | Example                     |
|----------|----------------------------------------|-----------------------------------|-----------------------------|
| **Basic**  | Smallest possible working OpMode     | Your very first OpMode            | BasicOpMode_Linear          |
| **Sensor** | How to read **one** specific sensor  | Testing new hardware              | SensorColor                 |
| **Robot**  | Simple drivetrain + one main idea    | Building a basic driver-controlled robot | RobotTeleopTank        |
| **Concept**| One focused advanced topic           | Vision, servos, PID, localization | ConceptVisionColorLocator   |

**Coach Advice:** Before copying anything, open 2–3 samples and read the comments at the top. In your notebook, note which prefix each uses and what it teaches.

---

## 3. How to Copy a Sample into Your Team Module (Step by Step)

Follow these steps the first few times.

1. In Android Studio, switch to **Project** view (Android mode).
2. Expand: `FtcRobotController > java > org.firstinspires.ftc.robotcontroller > external > samples`
3. Find the sample you want.
4. Right-click the `.java` file → **Copy**.
5. Expand your module: `Team00000 > java > org.firstinspires.ftc.teamcode`
6. Right-click the `teamcode` folder → **Paste**.
7. Rename the file with a clear name (e.g. `TeleOp_BasicDrive`, `Auto_Park`, `Test_ColorSensor`). Start with a capital letter. No spaces.
8. If Android Studio offers to refactor, accept it.

**After pasting:**
- First line must still read `package org.firstinspires.ftc.teamcode;`
- If you see red error lines: **Build > Clean Project**, then **Build > Rebuild Project**.

**Common Mistakes to Avoid**
- Forgetting to rename the file (you end up with duplicate "BasicOpMode_Linear" files).
- Accidentally editing inside `FtcRobotController`.
- Not committing to Git after a successful paste.

**Pro Tip:** For future reference, use the Quick Reference Card at the end of this guide — it is intentionally concise.

**Portfolio Prompt #2**  
Take a screenshot of your project tree after you have successfully pasted and renamed your first OpMode. Paste it into your notebook and label the file you created. Note one thing you would do differently next time.

---

## 4. Making Your OpMode Appear on the Driver Station

Near the top of the class you will see:

```java
@TeleOp(name="Template: Linear OpMode", group="Linear OpMode")
@Disabled
```

**Do this:**
1. Change `name="..."` to something your drivers will understand (e.g. `name="TeleOp - Tank Drive v1"`).
2. Set `group="TeleOp"`, `"Autonomous"`, or `"Test"`.
3. **Delete or comment out** the `@Disabled` line.

Save, rebuild, and your OpMode should appear when you select your team on the Driver Station.

---

## 5. Getting Help the Right Way (Mentor Bot & Human Mentors)

### Good Ways to Ask for Help
- “Which sample prefix should I start with to test our new distance sensor? Explain why without writing code.”
- “I added one motor to the Basic Linear OpMode. It spins the wrong direction. What questions should I ask myself?”
- “Explain what `waitForStart()` and `opModeIsActive()` actually do in simple terms.”

### Rules for Using the Mentor Bot (and Human Mentors)
1. Describe what you have already tried.
2. Never paste an entire unsolved problem and say “fix it for me.”
3. After receiving a hint, attempt it yourself before asking for more.
4. Bring your notebook and the bot’s answer to mentor check-ins.

**Portfolio Prompt #3**  
Write down one good question you could ask the Mentor Bot right now about the OpMode you just copied. Then actually ask it (following the rules above) and record the answer in your notebook.

---

## 6. Engineering Portfolio Expectations

This is not busy work. Judges look for evidence of your **design process, iteration, and data-driven decisions**. Documenting as you go is exactly what real engineers do.

**Your OpMode Portfolio Checklist (Minimum for every significant OpMode or change)**

- [ ] Screenshot of the Android Studio project tree showing your new/edited file
- [ ] Screenshot of the Driver Station showing your OpMode in the list
- [ ] Notebook entries answering the Embedded Portfolio Prompts (Prompt #1 in Section 1, Prompt #2 in Section 3, Prompt #3 in Section 5)
- [ ] Short reflection: What did this OpMode teach you? What will you improve or test next?
- [ ] (Mid-to-late season) Link to the specific Git commit or GitHub diff

---

## Quick Reference Card (Print or Bookmark This)

**Adding a New OpMode (Full details: Section 3)**

1. Choose the right sample using the Prefix Table (Section 2).
2. Copy the `.java` from `FtcRobotController > ... > samples`.
3. Paste into your `Team00000 > java > ... > teamcode` folder and rename clearly.
4. Edit `@TeleOp(name="Driver-friendly name", group="TeleOp")` and **delete** `@Disabled`.
5. If red errors appear: Build > Clean Project, then Build > Rebuild Project.
6. Test on the Driver Station, then complete the Portfolio Checklist (Section 6).

**Golden Rules — Memorize These**

- Your code lives only in your numbered team module. Never edit inside `FtcRobotController`.
- Commit to Git after every successful build or test.
- When asking for help (Mentor Bot or human): describe what you tried + ask a specific question. Never paste “fix my whole OpMode.”
- Document as you go. Notebook + screenshots + reflections = your engineering portfolio. Judges read it.

---

## Final Coach Message

Start by following this guide exactly: copy one simple sample, get it running on the Driver Station, and document it using the checklist. Then we will move to more advanced concepts together.

My code in `Team00000` is there so you can see how a more experienced programmer organizes and solves problems in real time. Use it to learn patterns — your code must be your own work.

I’m excited to see what you build.

— Coach Chris

**Document Version:** 2026 Season v2 – Streamlined (reduced repetition while preserving all learning objectives, embedded Portfolio Prompts, and tier-aware support). Always use the latest version from our shared resources.