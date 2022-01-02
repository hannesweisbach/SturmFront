# SturmFront
Fan-controller for indoor sports which adjusts fan speed according to your current heart rate.

I developed SturmFront as a low-cost, DIY-version of the well-known Wahoo® KICKR Headwind. In contrast to the KICKR Headwind, SturmFront does not have an integrated fan, but can be thought of like a smart plug, but instead of WiFi it uses ANT+ and instead of simply on/off-control SturmFront offers adjustable power output. This allows SturmFront to turn any fan into a smart fan.
Keith Wakeham developed also a DIY fan controller named [Maelstom](https://github.com/kwakeham/Maelstrom).

To modulate fan speed SturmFront uses phase control via a solid state relay (SSR).
During development and testing of SturmFront it soon became obvious to me that speed control linearly proportinal to heart rate is insufficient.
- First, I added a temperature input to the control software to compensate for colder air temperatures in winter and/or when my window is open. Temperature can be adjusted 5˚K intervals.
- Second, instead of linearly mapping the heart rate to control the SSR, fan speed can now be adjusted 5 bpm increments.

SturmFront uses bilinear mapping to interpolate between 5˚K and 5 bpm values.
