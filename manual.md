# CARDRONE manual

CARDRONE is a drone synth that listens to the room. WiFi signals bounce off walls, furniture and people on their way to your Cardputer. When someone moves, the bounces change, and CARDRONE turns those changes into slow shifts in the sound: a filter opening as you walk past, shimmer rising when the room gets busy, the drone settling when everyone sits still.

You don't need to play notes. Pick a sound, press play, and let the room do the rest.

## Quick start

1. Flash `cardrone.bin` (see the README) and power on.
2. The first time, a **network screen** appears. Pick **AUTO** at the top. CARDRONE listens on every WiFi channel for a few seconds and locks onto the busiest signal it hears. No password needed.
3. The drone is **silent until you press Space** (the boot screen and the footer remind you). Press Space to play, and again to stop. Or press a note key (see below) to start on that note. The boot screen's top-right corner shows the firmware version.
4. Walk around, wave your arms, sit still. Watch the waterfall and listen.
5. Press `i` any time for the key list, and `` ` `` (the key left of `1`) for options.

Plug in headphones and the speaker switches off by itself.

## The screen

**Header**, from left to right:

- the engine name, in its own color, so you can tell at a glance which one is playing; then the root note, octave and scale, e.g. `A2 MINOR`
- the network mode letter and channel: `A` auto, `J` join, `L` listen, so `A6` means AUTO on channel 6
- how many WiFi measurements arrive per second, e.g. `48Hz` (amber when nothing is coming in)
- the battery level (in the warning color below 20%)

**Footer:**

- a small `MOT` graph of recent motion
- `DSP %`: how hard the synth is working (keep it under ~80%)
- the volume, or `MUTED`, or `STOPPED`
- hints for the current page, and short messages such as `RANDOMIZED` or `BASELINE SET`

## Pages

Press **Tab** to cycle through four pages.

### 1. Waterfall

A picture of the WiFi signal over time. Each column is one of 52 slices of the WiFi channel, and new readings appear at the top and scroll down. Bright means strong, dark means weak. When the room is still, the waterfall forms steady vertical stripes; when people move, the stripes wobble and smear.

On the right are live meters for the eight **mod sources** (explained below). The dim bar behind each meter is a fading memory of its recent peak.

### 2. Bars

The same 52 slices as a live bar graph, so you can see the current "shape" of the signal. Dim ghosts show recent peaks. After you calibrate (key `e`), small marks show what the empty room looked like, so you can see how far the room has moved from it.

### 3. Params

The sixteen sound controls. Each has a bar for its setting and a bright tick showing where it actually is right now, after the room's modulation.

- `;` `.` select a param
- `,` `/` turn it down or up (hold to sweep)
- **Enter** (while playing) switches the selected param's effect on or off. Params of an effect that is off are greyed out and show `--`. SHIMMR has its own switch, separate from the reverb (it only sounds while the reverb is on).

### 4. Matrix (modulation)

This is where you decide what the room controls. There are 8 slots, and each one routes one **source** to one **param** with a **depth**:

```
1 MOT ▮▮▮ > CUTOFF   +35
```

means "motion opens the filter, by up to 35% of its range".

- `;` `.` pick a slot
- **Enter** (while playing) moves along the row: source → param → depth
- `,` `/` change whatever is highlighted. Depth goes from −100 to +100 in steps of 5. A negative depth works in reverse, so a rising source turns the param down.
- **Del** sets the slot's depth to 0 (off)

Routing changes glide: a new depth ramps in, and a slot moved to another source or param fades out and back in over about a second, so editing (or holding `r` to randomize the matrix) never jumps.

Several slots can point at the same param; their amounts add up. A depth of 20–40 is plenty for most things. How quickly the room's changes reach the sound is set by **MOD PROFILE** in the options; with the default, SMOOTH, they glide over about 1.5 seconds, so even a strong depth swells rather than jumps.

## The mod sources: what the room is telling you

Every WiFi packet your Cardputer receives carries a measurement of how the signal arrived: how strong each of the 52 slices was, and how they were shifted in time. CARDRONE boils those measurements down to seven sources, plus one LFO of its own.

All room sources run from 0 to 1, and they **adapt to your room**. After a few seconds, the calmest the room has been reads near 0 and the busiest near 1, and that scale keeps re-learning slowly. A quiet room drifts toward the low end on its own.

| Source | Plain-language meaning | Feels like |
|---|---|---|
| **MOT** (motion) | How much the room's WiFi "fingerprint" differs from what it was a moment ago. People walking, standing up, opening doors. | The main "someone is moving" sensor. Smooth and reliable. |
| **PHS** (phase) | How unsteady the timing of the signal is. Picks up movement in a different way from MOT. | Twitchier and less predictable; good for texture. |
| **FLX** (flux) | How much the signal changes from one packet to the next. Quick gestures and fast movement. | Fast and jittery; best at small depths or on effects. |
| **RSI** (signal strength) | How strong the WiFi signal is overall. Changes when you move the Cardputer or step between it and the router. | Slow and steady; good for big, gradual changes. |
| **LO / MID / HI** | How far the low, middle and top third of the 52 slices have moved from normal. Different movements disturb different slices, so these act like three sensors looking at the room from slightly different angles. | Like MOT, but each reacts to different things. After calibrating, they measure the difference from the empty room. |
| **LFO** | Not from the room: a slow wave from inside the synth. Shape and speed are in the options. | Regular, predictable movement. |

### Calibrating

Press `e` (or Options → CALIBRATE). A big **LEAVE THE ROOM** countdown gives you 5 seconds to step out. Then the screen shows **CALIBRATING** with a progress bar while CARDRONE records the empty room for about 150 WiFi readings (around 3 seconds at a good rate). When you come back you'll see `BASELINE SET`. Any key cancels, during the countdown or the capture. If it says *waiting for CSI*, no readings are arriving: check the network. From then on, **LO**, **MID** and **HI** measure "how different is the room from empty", which makes them steadier and more meaningful.

### Tips for good modulation

- More WiFi readings per second means smoother, more responsive sources. 30–50 Hz is great; under 10 Hz gets sluggish.
- **JOIN** mode gives the steadiest rate, because CARDRONE keeps the router busy answering pings.
- Put the Cardputer somewhere people pass between it and the router: that's where the room is most "visible".

## Sound engines

Press `1`–`7`. The sound fades over when you switch.

| Key | Engine | Sound | TIMBRE | SPREAD |
|---|---|---|---|---|
| `1` | **PWM** | Six pulse waves (root, fifth, sub) with slowly drifting widths. Warm and analog. | how much the pulse widths move | detune between voice pairs |
| `2` | **SAW** | Seven-voice supersaw plus a sine sub. Big and lush. | level of the side voices | detune |
| `3` | **FM** | Three FM bells (root, fifth, sub). Glassy and metallic. | brightness (FM depth) | slight beating between the bells |
| `4` | **ORG** | An organ of 16 partials whose loudness follows the live shape of the WiFi signal. The room literally shapes the tone. | how strongly the signal shapes the partials | stretches the partials apart (more bell-like) |
| `5` | **WAVE** | Wavetable. | scans 8 waves: sine → warm → triangle → saw → square → "ah" → nasal → glass | detune |
| `6` | **VOICE** | A choir: four voices through vocal formants. | the vowel: a → e → i → o → u | choir detune |
| `7` | **NOISE** | Five kinds of noise, not just colors: **dust** (vinyl crackle, embers), **wind** (broad gusts wandering on their own), **surf** (waves building and breaking every 6–12 s), **static** (choppy shortwave), **metal** (8-bit console noise ringing at the drone's pitch). Only metal is pitched. | which kind; in between, two neighbors blend | dust: density · wind: how broad the gusts are · surf: how bright the foam is · static: how crunchy |

### Voices

Press `p` to choose how many tones the drone stacks:

- **1**: just the root
- **2**: root and an interval above it: a fifth by default, the classic open drone
- **3**: root, interval and an octave below (the default)

Press `t` to change the interval: **fifth → fourth → octave → third**. The third follows your scale: minor in minor-type scales, major in major ones. The interval is saved with presets.

Every engine follows this. ORG shares its 16 partials among the voices, so at 1 voice it is one rich tone and at 3 a chord. NOISE ignores voices except metal, which rings at each voice's pitch. Fewer voices also leaves the synth more headroom (lower `DSP %`). The voice count is saved with presets.

### Performing

- **Freeze** (`f`): the room stops steering. Every room-driven modulation holds exactly where it is, and ORG's partials stop following the WiFi shape; the LFO keeps moving. The footer's `MOT` turns into an amber `FRZ`. Press `f` again to let the room back in. Handy when the room lands on a sound you love.
- **Auto drift** (`a`): the drone wanders by itself. Every 20–60 seconds the root glides to another note of the scale within a fifth of where you started; movement in the room makes it wander sooner. A `~` after the scale in the header shows it's on. Playing a note re-centers it.
- **Slow fade** (hold `q`): the drone fades out over a few seconds; let go and it swells back.
- **Undo** (`u`): steps back through the last 8 big changes: randomize (and the held mod roll), preset loads, notes, root and octave, scale, engine, voices, interval and effect switches. Small param nudges aren't recorded.

## Params

| Param | What it does |
|---|---|
| PITCH | Shifts the drone up to an octave up or down, always landing on a note of the chosen scale, and glides there slowly |
| CUTOFF | Filter brightness: low is dark and muffled, high is open and bright |
| RESO | Filter resonance: a singing peak at the cutoff |
| TIMBRE | The engine's main color (see the engine table) |
| SPREAD | Detune or width (see the engine table) |
| NOISE | Adds breathy noise (off by default) |
| LFO | Speed of the internal LFO |
| LEVEL | Output level of the drone |
| DRIVE | Analog-style saturation: warm even harmonics like a tube, getting darker the harder you push it |
| CRUSH | Unstable bit crusher: fewer bits, with the depth wandering and sputtering like a failing converter (the more CRUSH, the less stable) |
| RING | Ring modulation: metallic tremolo, from slow throb to clangorous |
| CHORUS | Width and shimmer of the chorus |
| D.TIME | Tape delay time (pitch bends when you change it, like real tape) |
| D.FB | Tape delay feedback: how long the echoes keep repeating |
| REVERB | Size and amount of the reverb |
| SHIMMR | Octave-up shimmer fed into the reverb: an angelic halo. Its own on/off switch (Enter on this row) |

Changes glide over a fraction of a second, so turning a knob, loading a preset or randomizing morphs instead of clicking.

## Keys

The bottom two letter rows are a one-octave keyboard, like a tracker:

```
 s  d     g  h  j        C#  D#      F#  G#  A#
z  x  c  v  b  n  m     C   D   E   F   G   A   B
```

A note sets the drone's root in the current octave (`9` `0` change the octave). The drone glides to it at the speed of your MOD PROFILE, so it steers rather than steps. Pressing a note while stopped starts the drone on that note.

| Key | Action |
|---|---|
| **Space** | play / stop, on any page |
| `z x c v b n m` | white notes C D E F G A B: the drone moves to that root in the current octave |
| `s d g h j` | black notes C# D# F# G# A# |
| **Shift** + note | the note an octave up; the next plain note drops back |
| `t` | interval: what voice 2 adds (see Voices) |
| `a` | auto drift on/off (see below) |
| `f` | freeze on/off (see below) |
| hold `q` | fade out slowly; let go to swell back |
| `u` | undo |
| **Fn** + `1`–`7` | switch an effect on or off from any page: 1 drive, 2 crush, 3 ring, 4 chorus, 5 delay, 6 reverb, 7 shimmer |
| **Enter** | on Params and Matrix it edits; elsewhere play / stop |
| **Tab** | next page |
| `1`–`7` | engine |
| `p` | voices: 1, 2 or 3 (see below) |
| `[` `]` | root note down / up a semitone |
| `9` `0` | octave down / up |
| `k` | next scale: minor, major, dorian, phrygian, pentatonic, fifths, chromatic |
| `r` | randomize params (pitch, noise and level stay put; shimmer usually stays out); **keep holding 3 s** (the footer counts down) to also randomize the modulation: every slot's source, param and depth, plus the LFO shape. The Matrix page then opens to show them. |
| `e` | calibrate the empty room (5 s countdown to leave, any key cancels) |
| `w` then `1`–`8` | save (write) a preset to the SD card |
| `l` then `1`–`8` | load a preset from the SD card |
| `-` `=` | volume |
| `` ` `` | options |
| `i` | help |
| `;` `.` | up / down |
| `,` `/` | adjust |
| **Del** | Matrix: switch the slot off |

## Options (`` ` ``)

Move with `;` `.`, change with `,` `/` or Enter, close with `` ` ``.

| Option | |
|---|---|
| AUDIO OUT | Mutes everything. (The headphone jack switches the speaker off in hardware; software can't separate them.) |
| VOLUME | 100% is the loudest the output can go without distorting |
| MOD PROFILE | how fast the room's changes reach the sound: **SHARP** (almost instant, for gestures), **NORMAL** (half a second), **SMOOTH** (1.5 s swells, the default) or **SLOW** (4 s tides). Pitch moves between scale notes at a matching pace. Remembered after power-off. |
| LFO SHAPE | sine, triangle, saw down, square, S&H (random steps), drift (smooth random wandering) |
| LFO RATE | from 0.02 Hz (one cycle per ~50 seconds) to about 5 Hz |
| THEME | green, yellow, red or blue phosphor, or B&W (grey, with warnings, calibration ticks and engine tags still in color); remembered after power-off |
| NETWORK | the network screen (see below) |
| CALIBRATE | record the empty room, after a 5 s countdown to leave |

The bottom line shows the lowest free memory since power-on, which helps with tracking down crashes.

## Network modes

Open from Options → NETWORK. Your choice is remembered and restored at the next power-on.

| Mode | How it works | When to use it |
|---|---|---|
| **AUTO** | Listens on all channels for ~3 seconds and locks onto the busiest WiFi device. If that device goes quiet for 8 seconds, it looks again. | Anywhere, no password. The easiest choice. |
| **JOIN** | Connects to your network and pings the router 50 times a second, so it always has fresh readings. | At home: steadiest and smoothest. Needs the password. |
| **LISTEN** | Listens to one network without connecting. | When you can't join but there's a busy network nearby. Depends on how much traffic it has. |

In the list, `;` `.` move, Enter picks, `` ` `` goes back. Networks with `*` need a password: type it, with Del to erase, and press Enter.

## Presets

Presets live on the SD card in `/cardrone/preset1.json` to `preset8.json`. A preset holds the engine, root, octave, scale, which effects are on, the LFO shape, all params and the whole matrix. Press `w` (save) or `l` (load), then a number; any other key cancels.

## Recipes

- **Breathing room:** PWM, MOT → CUTOFF +40, LFO (drift, slow) → CUTOFF +10, reverb high. The drone opens up when people move and closes when they settle.
- **Choir that reacts:** VOICE, PHS → TIMBRE +30 so the vowel drifts as the room changes, MOT → REVERB +25.
- **Weather in the room:** NOISE with TIMBRE around 0.3 (wind), MOT → TIMBRE +25 so walking in turns the wind into surf, and LFO (drift) → SPREAD +20.
- **The room's own voice:** ORG with TIMBRE high, so the WiFi signal's shape *is* the timbre. Calibrate first, then try HI → SHIMMR +30.

## If something seems off

- **Header shows `0Hz` in amber:** no readings are arriving. Try AUTO again, or JOIN your own network.
- **Mods barely move:** calibrate, then move more, or raise the depths. The sources adapt to the room, so constant motion eventually reads as "normal".
- **Mods move too much or feel jumpy:** lower the depths, or pick a slower MOD PROFILE. Two or three slots at 20–30 are usually plenty.
- **Mods feel sluggish:** pick MOD PROFILE → NORMAL or SHARP.
- **Crackles:** check `DSP %` in the footer. If it's near 100, switch off an effect.
- **After a crash**, the start screen says what happened (panic, watchdog or brownout). Note it down, with the free-memory number from the options page.
