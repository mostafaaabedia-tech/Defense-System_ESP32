# Laser Tripwire Base Alarm + Radar (ESP32)

A model military-base perimeter alarm. Four laser beams shine onto four LDRs, and a
servo-mounted ultrasonic radar sweeps the area in front of the base.

- **Laser breach:** the siren starts and **stays on** (latched) until a guard presses the **reset switch**
- **Radar contact:** two short chirps, and the object is shown on the radar scope (an early warning, no latched siren)
- **Two screens:** a status screen, and a 1.5" SH1107 **map screen** with a tactical map of the base and a live radar scope
- **Website** (GitHub Pages) and your phone get every alert within a second or two

```
firmware/base_alarm/base_alarm.ino   → upload to the ESP32
index.html                           → the website (GitHub Pages serves this)
oled-screens.png                     → what the two OLED screens look like
```

---

## Parts

| Part | Qty |
|---|---|
| ESP32 dev board (30 or 38 pin, **WROOM** module) | 1 |
| LDR (2-pin photoresistor) | 4 |
| Resistor, **10 kΩ** | 4 |
| Laser module (KY-008 or 2-wire 5 V laser) | 4 |
| Buzzer (active or passive) | 1 |
| Push button / switch (reset) | 1 |
| 1.5" OLED 128×128, status screen (SSD1327 or SH1107) | 1 |
| **1.5" OLED 128×128 SH1107**, map + radar screen | 1 |
| **Servo SG90** (or MG90S) | 1 |
| **HC-SR04** ultrasonic sensor | 1 |
| Resistors **1 kΩ + 2 kΩ** (for the HC-SR04 Echo pin) | 1 each |

---

## Wiring

### Pin summary

| ESP32 pin | Goes to |
|---|---|
| 32 / 33 / 34 / 35 | LDR zone 1 / 2 / 3 / 4 |
| 25 | Buzzer + |
| 27 | Reset switch (other leg to GND) |
| 21 / 22 | Status screen SDA / SCL |
| **16 / 17** | **Map screen (SH1107) SDA / SCL** |
| **13** | **Servo signal** |
| **26** | **HC-SR04 Trig** |
| **14** | **HC-SR04 Echo (through the divider!)** |
| 3.3V | LDRs, both screens' VCC |
| VIN / 5V | Lasers, servo, HC-SR04 VCC |
| GND | Everything's GND |

### LDRs (one voltage divider per zone)

```
 3.3V ──── LDR ────┬──── ESP32 GPIO (32 / 33 / 34 / 35)
                   │
                10 kΩ
                   │
 GND ──────────────┘
```

> These must be ADC1 pins (32–39). The ADC2 pins **stop reading analog values when WiFi is on**.

### Lasers

The lasers stay on all the time: KY-008 **S** → **VIN / 5V**, **–** → **GND**, middle pin empty.

### Status screen, buzzer, reset switch

| Part | Pin | ESP32 |
|---|---|---|
| Status OLED (I2C) | VCC / GND / SDA / SCL | 3.3V / GND / **21** / **22** |
| Buzzer | + / – | **25** / GND |
| Reset switch | legs | **27** / GND (internal pull-up, no resistor) |

If the status screen has **7 pins** (SPI): DIN→23, CLK→18, CS→5, DC→4, RST→15, then set `DISPLAY_TYPE 4`.

### Map screen (1.5" SH1107)

| SH1107 pin | ESP32 |
|---|---|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO **16** |
| SCL | GPIO **17** |

It is on its own I2C bus (pins 16/17), so it never clashes with the status screen even if
both screens use the same I2C address.

> GPIO 16/17 are free on normal ESP32 **WROOM** boards. On a WROVER board they are used by the extra RAM,
> so change `MAP_SDA` / `MAP_SCL` to two other free pins (for example 18 and 19).

**Only have one screen?** Set `#define MAP_SCREEN 0`. The status screen then switches between the
status page and the map page every 5 seconds, and jumps to the radar scope when something is detected.

### Radar: servo + HC-SR04

```
 Servo SG90                HC-SR04
  brown  → GND              VCC  → VIN / 5V
  red    → VIN / 5V         GND  → GND
  orange → GPIO 13          Trig → GPIO 26
                            Echo ──[1 kΩ]──┬── GPIO 14
                                           │
                                         [2 kΩ]
                                           │
                                          GND
```

> **The divider on Echo is important.** The HC-SR04 sends back 5 V, and the ESP32 pins only take 3.3 V.
> The 1 kΩ + 2 kΩ pair brings it down to about 3.3 V.

Glue the HC-SR04 onto the servo horn so it turns with it. Mount the servo so that **90° points straight out
from the base** (0° = right, 180° = left). That way the angles on the screens and the website match real life.

If the servo shakes or the ESP32 resets when it moves, the USB port can't supply enough current:
power the servo from a separate 5 V supply (with GNDs connected together), or add a 470 µF capacitor across the servo's 5 V and GND.

---

## Upload the firmware

1. Arduino IDE → Boards Manager → install **esp32 by Espressif**.
2. Library Manager → install **U8g2** (by olikraus). No servo library is needed.
3. Open `firmware/base_alarm/base_alarm.ino` and edit the top:
   ```cpp
   const char* WIFI_SSID  = "YOUR_WIFI_NAME";
   const char* WIFI_PASS  = "YOUR_WIFI_PASSWORD";
   const char* NTFY_TOPIC = "base-alarm-CHANGE-ME-4821";   // make up your own secret name
   #define DISPLAY_TYPE 1        // status screen type, see below
   #define MAP_SCREEN 1          // 1 = second SH1107 screen, 0 = only one screen
   #define BUZZER_PASSIVE 0      // 0 = active buzzer, 1 = passive buzzer
   #define RADAR_ENABLED 1       // 0 if you haven't built the radar yet
   const int RADAR_RANGE_CM = 50;   // how far the radar looks
   ```
4. Board: **ESP32 Dev Module** → Upload. Open Serial Monitor at 115200 to see light readings and radar contacts.

**Which DISPLAY_TYPE?** `1` SSD1327 I2C · `2` SSD1327 I2C (try if picture is shifted) · `3` SH1107 I2C · `4` SSD1327 SPI (7 pins).
If the map screen looks shifted or garbled, change `MAP_SCREEN_VARIANT` to 2, then 3.

**Which buzzer?** If it beeps when you connect it straight to 3.3V it's *active* (`0`). If it only clicks, it's *passive* (`1`).

---

## How it works

| What happens | What the base does |
|---|---|
| Power on | Waits 5 s, measures each beam's brightness, radar learns the area, then **ARMED** |
| Beam broken for 40 ms | **ALARM**: siren, flashing screens, phone + website alert |
| Another beam broken during the alarm | "Second breach" alert for that zone too |
| New object in front of the radar | **RADAR CONTACT**: 2 chirps, contact on both screens + website (max 1 alert a minute) |
| Object leaves | "Radar clear" message |
| Every 15 minutes | Quiet status report so the website knows the base is online |
| Press reset | Siren off, 5 s countdown, re-armed (keeps the old calibration) |
| **Hold** reset 3 s | Two beeps, re-measures the lasers **and re-learns the radar area** |
| A laser isn't hitting its LDR at calibration | That zone shows **OFF** and is skipped |

### How the radar decides what is an intruder

The servo sweeps 0° → 180° → 0° (about 6 s for a full round). During the first two sweeps after
calibration it **learns** how far away things normally are in every direction (the model buildings, walls, the table edge).
After that, it only reports something that is **at least 6 cm closer than normal** and **inside the radar range**, and only if it
shows up in two neighbouring directions, which filters out single bad echoes.

So: after you hold reset, **step back out of the radar's view** until the map screen says `RDR CLEAR` instead of `RDR LEARN`.
If you move a building on the model, hold reset 3 s so the radar learns the new layout.

### The two screens

**Status screen:** big state word (ARMED / INTRUDER / ARMING), a live light bar for each laser (use it to aim them),
and a footer showing the radar status.

**Map screen (SH1107):**
- Top left: a tactical map of the base: fence, 4 watchtowers, the 4 laser walls, HQ, helipad, barracks, motor pool,
  and the radar dish with a line showing where it is pointing right now.
  Beams are solid when on watch, thick and blinking when broken, dotted when bypassed, dashed while arming.
- Top right: each zone's status and the radar status.
- Bottom: a radar scope with range rings, the live sweep line, dots for new objects, and a circle on the main contact
  with its distance and angle.

---

## Put the website on GitHub

1. Create a new repository, e.g. `base-alarm`.
2. Upload `index.html` (and the `firmware` folder, `README.md` if you want).
3. In `index.html`, near the top of the `<script>`, set
   `const DEFAULT_TOPIC = "...";` to the **same** name as `NTFY_TOPIC`.
4. Repo **Settings → Pages → Source: Deploy from a branch → main / (root) → Save**.
5. After a minute it's live at `https://YOUR-USERNAME.github.io/base-alarm/`.

On the page, press **Turn on alerts** once so the browser can play sounds and show notifications.
**Breach drill** and **Radar drill** send a simulated alert so you can test the page and your phone without touching
the hardware (they always show as drills, never as real alerts).

### What the command screen shows

| Part | What it tells you |
|---|---|
| Condition banner + lamps | **Secure**, **Caution** (radar contact, arming, or base not reporting) or **Breach** with the zones and how long ago |
| Tactical map + radar | The base with 4 watchtowers; each laser runs tower to tower along a wall, broken beams turn red. The radar sweeps from the radar building (facing north, 3 px = 1 cm like the scale bar) and its contacts appear right on the map, fading over 3 minutes. Below the map: radar status, current contact, last contact, contacts today |
| Perimeter zones | Each zone's beam strength (% of its calibrated brightness) at the last report, and its status |
| Base link | Whether the ESP32 is online, last report time, Wi-Fi signal, uptime, breaches today |
| Incident log | Every power-on, arm, breach, radar contact, radar clear, reset and drill from the last 12 hours |

If no report arrives for 17 minutes the page switches to **Base not reporting**, and after 35 minutes to **Contact lost**,
so you can tell the difference between "all quiet" and "the base is switched off".

On the website map the radar faces north (90° = toward zone 1). The sweeping line shows how the radar moves. It isn't the servo's exact live position
(sending that every 50 ms would use up the free message limit in minutes), but the dots are real contacts reported by the base.

### Alerts on your phone, even with the website closed

The ESP32 sends alerts through the free service **ntfy.sh** (GitHub Pages can only show
pages, it can't receive messages by itself, so ntfy is the go-between).
Install the **ntfy** app (Android / iPhone), tap **+**, and subscribe to your topic name.
Intruder alerts arrive as max-priority notifications, radar contacts as high priority.

The free ntfy.sh service limits how many messages you can send per day, so the base sends at most 60 radar messages a day
and a status report every 15 minutes.

> Anyone who knows your topic name can read or send messages on it, so pick a name that's hard to guess.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| Alarm goes off by itself | Room too bright / laser barely hitting. Add tubes on LDRs, re-aim, hold reset 3 s. Or lower `TRIP_RATIO` (e.g. 0.45) |
| Breaking the beam does nothing | Check the zone isn't **OFF**. Open Serial Monitor: light value should drop a lot when you block it |
| Zone always OFF | Laser not hitting the LDR, or the resistor/LDR wired the wrong way round (swap them) |
| Map screen black | Check SDA→16, SCL→17, VCC on 3.3V. Try `MAP_SCREEN_VARIANT` 2 or 3 |
| Radar never sees anything | Check the Echo divider and Trig/Echo aren't swapped. Serial Monitor should print "Radar contact" when you put your hand in front |
| Radar contacts all the time | You were in view while it learned. Hold reset 3 s and step away. Or raise `RADAR_MARGIN_CM` to 10 |
| Servo jitters / ESP32 restarts | Servo needs more current: separate 5 V supply or a 470 µF capacitor |
| Website says "Awaiting base" | Topic names don't match, or ESP32 not on WiFi (status screen shows NO NET). The alarm still works offline |
