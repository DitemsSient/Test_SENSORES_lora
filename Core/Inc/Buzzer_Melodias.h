/**
 * @file    Buzzer_Melodias.h
 * @brief   Predefined melody arrays for use with Buzzer_PlayMelody().
 *
 * @details Contains BuzzerNote_t arrays ready to pass to Buzzer_PlayMelody().
 *          Negative duration values indicate a dotted note (×1.5 duration).
 *          Include this file after "Buzzer.h".
 *
 * @date    March 27, 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef BUZZER_MELODIAS_H
#define BUZZER_MELODIAS_H

#include "Buzzer.h"

/* ====================  PREDEFINED MELODIES  =============================== */

/* The Pink Panther Theme — Henry Mancini. Recommended tempo: 120 BPM */

static const BuzzerNote_t pink_panther[] = {

    /* Intro (wait measure) */
    { SILENCE,  HALF        },
    { SILENCE,  QUARTER     },
    { SILENCE,  EIGHTH      },
    { Ds4,      EIGHTH      },

    /* Main theme — Phrase 1 */
    { E4,       -QUARTER    },      /* E4 dotted                          */
    { SILENCE,  EIGHTH      },
    { Fs4,      EIGHTH      },
    { G4,       -QUARTER    },      /* G4 dotted                          */
    { SILENCE,  EIGHTH      },
    { Ds4,      EIGHTH      },

    { E4,       -EIGHTH     },      /* E4 dotted                          */
    { Fs4,      EIGHTH      },
    { G4,       -EIGHTH     },
    { C5,       EIGHTH      },
    { B4,       -EIGHTH     },
    { E4,       EIGHTH      },
    { G4,       -EIGHTH     },
    { B4,       EIGHTH      },

    { As4,      HALF        },
    { A4,       -SIXTEENTH  },
    { G4,       -SIXTEENTH  },
    { E4,       -SIXTEENTH  },
    { D4,       -SIXTEENTH  },

    /* Phrase 2 (varied repeat) */
    { E4,       HALF        },
    { SILENCE,  QUARTER     },
    { SILENCE,  EIGHTH      },
    { Ds4,      QUARTER     },

    { E4,       -QUARTER    },
    { SILENCE,  EIGHTH      },
    { Fs4,      EIGHTH      },
    { G4,       -QUARTER    },
    { SILENCE,  EIGHTH      },
    { Ds4,      EIGHTH      },

    { E4,       -EIGHTH     },
    { Fs4,      EIGHTH      },
    { G4,       -EIGHTH     },
    { C5,       EIGHTH      },
    { B4,       -EIGHTH     },
    { G4,       EIGHTH      },
    { B4,       -EIGHTH     },
    { E5,       EIGHTH      },

    /* Bridge */
    { Ds5,      WHOLE       },      /* Ds5 — long note                    */

    { D5,       HALF        },
    { SILENCE,  QUARTER     },
    { SILENCE,  EIGHTH      },
    { Ds4,      EIGHTH      },

    /* Phrase 3 (restatement) */
    { E4,       -QUARTER    },
    { SILENCE,  EIGHTH      },
    { Fs4,      EIGHTH      },
    { G4,       -QUARTER    },
    { SILENCE,  EIGHTH      },
    { Ds4,      EIGHTH      },

    { E4,       -EIGHTH     },
    { Fs4,      EIGHTH      },
    { G4,       -EIGHTH     },
    { C5,       EIGHTH      },
    { B4,       -EIGHTH     },
    { E4,       EIGHTH      },
    { G4,       -EIGHTH     },
    { B4,       EIGHTH      },

    { As4,      HALF        },
    { A4,       -SIXTEENTH  },
    { G4,       -SIXTEENTH  },
    { E4,       -SIXTEENTH  },
    { D4,       -SIXTEENTH  },

    /* Coda */
    { E4,       -QUARTER    },
    { SILENCE,  QUARTER     },

    { SILENCE,  QUARTER     },
    { E5,       -EIGHTH     },
    { D5,       EIGHTH      },
    { B4,       -EIGHTH     },
    { A4,       EIGHTH      },
    { G4,       -EIGHTH     },
    { E4,       -EIGHTH     },

    { As4,      SIXTEENTH   },
    { A4,       -EIGHTH     },
    { As4,      SIXTEENTH   },
    { A4,       -EIGHTH     },
    { As4,      SIXTEENTH   },
    { A4,       -EIGHTH     },
    { As4,      SIXTEENTH   },
    { A4,       -EIGHTH     },

    { G4,       -SIXTEENTH  },
    { E4,       -SIXTEENTH  },
    { D4,       -SIXTEENTH  },
    { E4,       SIXTEENTH   },
    { E4,       SIXTEENTH   },
    { E4,       HALF        },
};

/* Ode to Joy (simplified) — Beethoven. Recommended tempo: 120 BPM */

static const BuzzerNote_t ode_to_joy[] = {
    /* Measure 1 */
    { E4,   QUARTER },
    { E4,   QUARTER },
    { F4,   QUARTER },
    { G4,   QUARTER },

    /* Measure 2 */
    { G4,   QUARTER },
    { F4,   QUARTER },
    { E4,   QUARTER },
    { D4,   QUARTER },

    /* Measure 3 */
    { C4,   QUARTER },
    { C4,   QUARTER },
    { D4,   QUARTER },
    { E4,   QUARTER },

    /* Measure 4 */
    { E4,   HALF    },
    { D4,   HALF    },

    /* Measure 5 */
    { E4,   QUARTER },
    { E4,   QUARTER },
    { F4,   QUARTER },
    { G4,   QUARTER },

    /* Measure 6 */
    { G4,   QUARTER },
    { F4,   QUARTER },
    { E4,   QUARTER },
    { D4,   QUARTER },

    /* Measure 7 */
    { C4,   QUARTER },
    { C4,   QUARTER },
    { D4,   QUARTER },
    { E4,   QUARTER },

    /* Measure 8 */
    { D4,   HALF    },
    { C4,   HALF    },
};

/* Short alert / notification melody. Recommended tempo: 160 BPM */

static const BuzzerNote_t alert[] = {
    { C5,       EIGHTH  },
    { E5,       EIGHTH  },
    { G5,       EIGHTH  },
    { SILENCE,  EIGHTH  },
    { G5,       QUARTER },
};

/* Boot OK — single confirmation beep. Recommended tempo: 140 BPM */

static const BuzzerNote_t boot_ok[] = {
    { C5,       SIXTEENTH },
    { SILENCE,  SIXTEENTH },
    { G5,       EIGHTH    },
};

#endif /* BUZZER_MELODIAS_H */
