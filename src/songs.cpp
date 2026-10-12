/*
 * C wrapper around the course's song data.
 * song_def.h defines the songs as global C++ objects, so it must be included
 * in exactly one .cpp file: this one.
 */
#include "song_def.h"
#include "songs.h"

struct SongInfo {
    Song *song;
    int   notes;   /* real size of the note array */
    int   beats;   /* real size of the beat array */
};

#define INFO(S, N, B) { &(S), (int)(sizeof(N) / sizeof((N)[0])), (int)(sizeof(B) / sizeof((B)[0])) }

static const SongInfo songs[SONG_COUNT] = {
    INFO(FUR_ELISE,               Fur_Elise_note,                      Fur_Elise_beat),
    INFO(CANNON_IN_D,             Canon_In_D_note,                     Canon_In_D_beat),
    INFO(MINUET_IN_G_MAJOR,       Minuet_In_G_major_note,              Minuet_In_G_major_beat),
    INFO(TURKISH_MARCH,           Turkish_March_note,                  Turkish_March_beat),
    INFO(NOCTRUNE_IN_E_FLAT,      Nocturne_in_E_flat_note,             Nocturne_in_E_flat_beat),
    INFO(WALTZ_NO2,               Waltz_No2_note,                      Waltz_No2_beat),
    INFO(NOCTRUNE_IN_C_SHARP_MAJOR, Nocturne_in_C_sharp_minor_note,    Nocturne_in_C_sharp_minor_beat),
    INFO(SYMPHONY_NO40,           Symphony_No40_note,                  Symphony_No40_beat),
    INFO(SYMPHONY_NO5,            Symphony_No5_note,                   Symphony_No5_beat),
    INFO(EINE_KLEINE_NACHTAMUSIK, Eine_Kleine_Nachtamusik_note,        Eine_Kleine_Nachtamusik_beat),
};

extern "C" {

int song_length(int s)
{
    const SongInfo &x = songs[s];
    int n = x.song->length;
    if (x.notes < n) n = x.notes;
    if (x.beats < n) n = x.beats;
    return n;
}

float song_tempo(int s)             { return songs[s].song->tempo; }
const char *song_name1(int s)       { return songs[s].song->name1.c_str(); }
const char *song_name2(int s)       { return songs[s].song->name2.c_str(); }
float song_note(int s, int i)       { return songs[s].song->note[i]; }
float song_beat(int s, int i)       { return songs[s].song->beat[i]; }

void song_counts(int s, int *declared, int *notes, int *beats)
{
    *declared = songs[s].song->length;
    *notes    = songs[s].notes;
    *beats    = songs[s].beats;
}

} /* extern "C" */