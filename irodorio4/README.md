# irodorio4 and mido4

> this is horridly unfinished and still (somewhat) in-development
>
> i currently am waiting to get the motivation to decouple the driver portion
> of the python poc into a zig library that will become irodorio4, whereas the
> midi stuff goes into mido4

irodori4 is a userspace driver and hardware abstraction layer for a chunithm
controller running (or emulating) the sega io4 arcade firmware  
(or, for tasollers:
[bottersnike's tasoller/host-aprom firmware on gitea.tendokyu.moe](https://gitea.tendokyu.moe/tasoller/host-aprom/))

mido4 is a userspace midi device driver around irodorio4 that turns a chunithm
controller into a tuba-like midi device

currently, mido4 is a single proof of concept python file that:

1. talks to a chunithm controller
    - handles initialisation and reset sequences
    - reads slider sensors at 33hz via COM/serial
      (half of the actual chunithm slider input reports, 66hz/15ms)
    - reads air tower sensors at 75hz via HID/usb
      (vs IO4's 125hz/8ms interval for input reports)
    - input readers are on their own threads (which didn't improve overhead anyways)
    - sets slider leds through serial commands
    - does NOT set air tower leds, they're set through the LED 15093 protocol,
      which i've yet to wrap my head around how to do so

2. outputs midi events with a tuba-like interface using a digital midi port
    - maps slider touch positions to musical fingerings similar to a tuba
    - uses air sensors for octave selection
    - sensor pressure is translated into dynamics
      (poorly, i'm currently only using repeated midi on notes, but turns out
      rtmidi can send change events)

## an unfinished guide to playing the mido4

### a heads up

if playing the mido4 tabletop, please face the air towers away from you,
as this was intended to be played propped similarly like a harp

if this inconveniences you, feel free to change the code yourself until
i eventually program a tabletop mode once i move the userspace hal driver
portion of mido4 out into a zig library (irodorio4)

### fingering

there are five valves:

- H
- 1
- 2
- 3
- 4

or, if you're playing the mido4 on the table-top,
from **right**-to-left: `H1234`

where, 1-4 act like as they would for a tuba,
but H is activated for the higher notes in the chromatic scale

| note | fingering   |
| ---- | ----------- |
| Bb   | `-____` (0) |
| B    | `-123_`     |
| C    | `-1_3_`     |
| C#   | `-_23_`     |
| D    | `-12__`     |
| Eb   | `-1___`     |
| E    | `-_2__`     |
| F    | `H____`     |
| F#   | `H_23_`     |
| G    | `H12__`     |
| G#   | `H1___`     |
| A    | `H_2__`     |

### playing notes and dynamics

the five valves have two halves, facing towards you is the 'inward' valve,
and facing away from you is the 'outward' valve

or as another notation table:

| inward | outward |
| ------ | ------- |
| Hi     | Ho      |
| 1i     | 1o      |
| 2i     | 2o      |
| 3i     | 3o      |
| 4i     | 4o      |

to initiate a fingering without playing the note,
you can use the outward half of the valves

when you want to play the note, move any finger from an outward half
to an inward half of the valve

this is useful when you want to ease into a note: you would position your
fingers in the desired fingering on the outward valves, then move either one
or all your fingers onto the inward half of the valves

### layout

```
LAYOUT (GROUND32)
... THIS WAY UP TO A CHUNITHM SCREEN (TASOLLER LOGO FOR TASOLLERS)
... LEFT <--------> RIGHT
... ... ground32[1] ground32[3] ground32[5] <---> ground32[31]
... ... ground32[0] ground32[2] ground32[4] <---> ground32[30]
... THIS WAY DOWN TO A HUMAN

LAYOUT (AIR6)
... THIS WAY UP TO THE SKY
... ... air6[5]
... ... air6[4]
... ... air6[3]
... ... air6[2]
... ... air6[1]
... ... air6[0]
... THIS WAY DOWN TO THE GROUND

VALVE MAPPINGS
... ---- "INWARD"    "OUTWARD"
... H -> ground32[9] ground32[8]
... 1 -> ground32[7] ground32[6]
... 2 -> ground32[5] ground32[4]
... 3 -> ground32[3] ground32[2]
... 4 -> ground32[1] ground32[0]
```
