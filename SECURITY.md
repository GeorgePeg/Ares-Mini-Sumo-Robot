# Security Policy

## Supported Versions

Here you can see which version of our robot you can find in the repository. We sugest you should use the ones that are marked with ✅. The table bellow refers to the .ino files inside the 📁Ares_codes.

| Version | Recommended for use| Description |
| ------- | ------------------ |-------------|
| 1.3.0   |✅|⚠️Referee Mode (You must have the referee remote control to test it)|
| 1.2.4   | ✅|Waiting  Strategy|
| 1.2.3   | ✅|RC5 remote control with fuzzyAttack()|
| 1.2.2   | ✅|RC5 remote control (corrected)|
| 1.2.1   | ❌|⚠️ RC5 remote control (errors during testing)|
| 1.1.0   | ✅|Debugging using BLE protocol|
| 1.0.0   | ✅|Simple Version|

## Reporting a Vulnerability
Security and integrity is our main thing. Therefore, if you discover a vulnerability (e.g. a bug inside one of our codes that could cause hardware malfunction, battery issues, or safety hazards and etc.), please follow these steps:
<ul>
  <li><strong>Please don't open a public issue.</strong>Disclosing a vulnerability publicly before a fix is available, it can put other users and their hardware at risk.</li>
  <li>Send an email to: gpegiazis@gmail.com with the subject: "Security Vulnerability Report for Ares Mini Sumo repository.</li>
  <li>Please include a detailed decription of the issue, steps to reproduce it, and the potential impact on the robot.</li>
</ul>
<strong>We will acknowledge receipt of your report within 72 hours.</strong>
We will work to resolve the issue as quickly as possible.
Once the fix is released, we will provide appropriate credit to you in our CHANGELOG (unless you prefer to remain anonymous).We will keep you updated on the progress of the fix.
<br>
<h2><strong>Hardware Safety</strong></h2>
Since this project involves physical hardware (motors, batteries, and sensors), please exercise caution:
<ul>
<li>Source Code: Do not upload code to your robot from untrusted sources or forks that you have not personally verified.</li>
<li>Dependencies: Ensure that any third-party libraries used (e.g., for motor drivers or IR sensors) are kept up to date.</li>
<li>Physical Risks: Be aware that software bugs in hardware control can lead to physical damage (e.g., stalled motors or battery overheating). Always test new code in a controlled environment.</li>
</ul>

