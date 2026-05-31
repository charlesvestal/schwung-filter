# schwung-filter

A multi-mode filter audio FX module for Schwung / Ableton Move. It provides a state-variable filter (lowpass, highpass, bandpass, and related modes) for use in the Signal Chain.

See the design doc at `schwung/docs/plans/2026-05-31-multimode-filter-design.md` and the milestone-1 plan at `schwung/docs/plans/2026-05-31-multimode-filter-milestone-1.md`.

Milestone 1 (SVF foundation) complete: clean state-variable filter with all six
modes, resonance to self-oscillation, log-space cutoff smoothing, envelope-free.
Planned next: envelope follower + tempo-synced LFO (M2), and analog model skins —
Moog ladder, Oberheim SEM, Prophet, Roland (M3).
