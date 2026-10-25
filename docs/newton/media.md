# Newton media provenance

The bundled hand and press figures use Newton's actual link poses and per-taxel force
arrays captured at the same simulation steps. A separate RTX renderer keeps
physics stopped and verifies each body pose, scale, mesh offset and vertex
against the pre-export physical geometry at every frame. The color scale is
shared across the entire clip; poses and forces are never interpolated.

[The machine-readable evidence](../media/newton-examples.json) includes capture
and render hashes, source-file hashes, package versions, the requested/completed
tick counts and the original physics-report hashes. Native camera output was
inspected for the hand and indenter rather than accepted only by file presence.

The bundled captures use the earlier 738-taxel layout recorded in their input
hashes. They illustrate the workflow; validate current 740-taxel results with
fresh captures.

The hand video covers four complete known-load visits at simulation speed;
the press video contains the third 2.5 s physical cycle at 2× slower playback.
Each video has phase captions and before/contact/final stills. These are Newton
results; PhysX media retain their own capture provenance.
