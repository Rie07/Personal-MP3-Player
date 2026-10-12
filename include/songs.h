#ifndef SONGS_H
#define SONGS_H

/* Plain-C view of the course's C++ song data (song.h / song_def.h).
 * Only songs.cpp includes the C++ headers; the rest of the firmware stays C. */

#ifdef __cplusplus
extern "C" {
#endif

#define SONG_COUNT        10   /* songs defined in song_def.h                 */
#define SONG_SELECTABLE    8   /* reachable with three buttons (binary 0..7)  */

/* Number of usable notes: the smaller of the declared length and the real
 * size of the note and beat arrays (protects against a wrong length). */
int          song_length(int s);
float        song_tempo(int s);
const char  *song_name1(int s);
const char  *song_name2(int s);

/* Note value is the PERIOD in milliseconds (Do = 3.822 ms = 261.6 Hz).
 * 0 means a rest. */
float        song_note(int s, int i);
float        song_beat(int s, int i);

/* For debugging: declared length, real note count, real beat count. */
void         song_counts(int s, int *declared, int *notes, int *beats);

#ifdef __cplusplus
}
#endif

#endif /* SONGS_H */
