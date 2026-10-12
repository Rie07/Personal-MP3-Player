# Personal MP3 Player (BCA182 Laboratory Activity 2)

A small MP3-style music player for the **RT-Thread RT-Spark** development board (STM32F407),
built with **PlatformIO**, the **STM32Cube HAL** (no Arduino) and **FreeRTOS**.

- Course: BCA182 Embedded Systems Programming, Mindanao State University - Iligan Institute of Technology
- Authors: Renato G. Beroy, Sherie Grace L. Lupoz, Binyusoph P. Sansarona
- Board: RT-Spark (STM32F407ZGT6, 168 MHz, 8 MHz crystal)

## What it does

- Plays the songs defined in `song_def.h` through the board's ES8388 audio codec (headphone jack).
- Shows the song name, the player state and the volume on the on-board 240x240 LCD.
- Lets you browse the songs with the on-board keys, with a 5 second confirmation window.
- Shows the player state on three LEDs.
- Volume is controlled with a potentiometer.
- Prints the instructions and the song list over UART when it starts.

## Controls

All controls use the four on-board keys.

| Key | Action |
|---|---|
| LEFT / RIGHT | Choose the previous / next song. This starts a 5 second window. |
| UP | Confirm the chosen song, and it plays from the start. With no song waiting, UP stops / plays. |
| DOWN | Faster tempo (wraps back to the slowest after the fastest). |
| Potentiometer | Volume. |

If you do nothing for 5 seconds after choosing a song, the choice is cancelled and the current song carries on.

| LED | Meaning |
|---|---|
| Yellow | Playing |
| Red | Paused / stopped |
| Green | Choosing a song (waiting for confirmation) |

## Songs

1. Fur Elise - Beethoven
2. Canon in D - Pachelbel
3. Minuet in G major - Bach
4. Turkish March - Mozart
5. Nocturne in E flat - Chopin
6. Waltz No. 2 - Shostakovich
7. Nocturne in C sharp minor - Chopin
8. Symphony No. 40 - Mozart
9. Symphony No. 5 - Beethoven
10. Eine Kleine Nachtmusik - Mozart

## Software design

Three FreeRTOS threads, named as in the handout, plus a start-up task:

| Thread | Job |
|---|---|
| `update_lcd_leds_thread` | Draws the LCD and drives the LEDs. |
| `polling_buttons` | Reads the keys (debounced) and runs the song choice / confirm logic. |
| `adjust_volume` | Reads the potentiometer, sets the codec volume, prints the UART instructions. |
| `audio_task` | Sets up the codec and starts the audio stream (then ends). |

- The **Ticker** that steps through the notes is a FreeRTOS software timer.
- The **Timeout** for the 5 second confirmation is a one-shot FreeRTOS software timer.
- The LCD is protected by a mutex (`lcd_mutex`).
- Audio: I2S3 with DMA in circular mode; the half/full-transfer callbacks refill the buffer with a triangle wave at the current note's pitch.
- The idle task executes `WFI` to sleep between interrupts.
- The note values in `song_def.h` are periods in milliseconds, so the pitch is `1000 / value` Hz. A value of 0 is a rest.

## Pin map

| Part | Pin |
|---|---|
| Yellow LED | PB14 |
| Green LED | PA1 |
| Red LED | PA8 |
| Potentiometer (ADC3 channel 4) | PF6 |
| Keys: UP / DOWN / LEFT / RIGHT | PC5 / PC1 / PC0 / PC4 (active low) |
| UART (USART1, 115200 baud, via ST-Link USB) | PA9 (TX), PA10 (RX) |
| LCD (ST7789, FSMC 8-bit) | D0-D7 = PD14, PD15, PD0, PD1, PE7-PE10; RS = PD13 (A18); RD = PD4; WR = PD5; CS = PG10 (NE3); RST = PD3; BL = PF9 |
| Audio codec control (I2C2) | PF1 (SCL), PF0 (SDA), address 0x10 |
| Audio codec data (I2S3) | PC7 (MCK), PA15 (WS), PB3 (CK), PB5 (SD) |

LEDs and the potentiometer are wired on a breadboard; everything else is on the board.

## Build and upload

1. Install VS Code with the PlatformIO extension.
2. Open this folder, then **Build** (check mark in the bottom bar).
3. Connect the board's **ST-Link** USB port and **Upload** (arrow). If Windows shows an error on the ST-Link Debug device, install ST's ST-Link driver first.
4. Optional: open the serial monitor on the board's COM port at 115200 baud to see the instructions.

`platformio.ini` uses the `black_f407zg` board definition, `framework = stm32cube`, `upload_protocol = stlink`,
and the FPU flags (`-mfpu=fpv4-sp-d16 -mfloat-abi=hard`). `fpu_link.py` adds the same flags to the linker.

## Project layout

```
include/   FreeRTOSConfig.h, lcd.h, lcd_font.h, songs.h, song.h, song_def.h
lib/       FreeRTOS-Kernel (Cortex-M4F port, heap_4)
src/       main.c (player), lcd.c (ST7789 driver), songs.cpp (C wrapper for the C++ song data)
ref/       reference files from the handout (not compiled)
```

`song.h` and `song_def.h` are the course-provided files. Because they are C++, only `songs.cpp` includes them
and exposes the data to the C code through `songs.h`.

## Differences from the handout

- Songs are chosen with LEFT / RIGHT instead of a binary code on buttons 2-4, so all 10 songs are reachable (3 buttons would only reach 8).
- The board has four usable keys, so there is no separate USER_BUTTON: UP stops / plays when no song is waiting for confirmation.
- Three single LEDs are used instead of an RGB LED (yellow stands in for blue).
- The handout's original `main.cpp` timing code was not available, so the note length is `beat x tempo x scale` with a tempo scale tuned by ear (default 6.0, DOWN key changes it).

## Credits

- Course files `song.h` / `song_def.h` by the course staff.
- The ES8388 start-up register sequence follows the idea of the RT-Thread ES8388 driver (Apache-2.0).
- The LCD font table was generated from DejaVu Sans Mono Bold (free licence).