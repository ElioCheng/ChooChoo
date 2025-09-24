#ifndef __MARKLIN_EDGE_RESISTANCE_LIST_H__
#define __MARKLIN_EDGE_RESISTANCE_LIST_H__

#ifndef TRACK_RESISTANCE
#define TRACK_RESISTANCE(track, from_node, to_node, coefficient)
#endif

// Track resistance coefficients configuration
// Format: TRACK_RESISTANCE(track_type, from_node_name, to_node_name, resistance_coefficient)
//
// Resistance coefficient is a fixed-point number with scale factor 1000:
// - 1000 = normal resistance (1.0)
// - 500 = half resistance (0.5) - less resistance, easier movement
// - 1500 = 1.5x resistance (1.5) - more resistance, harder movement
// - 2000 = double resistance (2.0) - much more resistance

// Track A resistance configurations
TRACK_RESISTANCE(MARKLIN_TRACK_TYPE_A, "BR8", "D9", 1200)
TRACK_RESISTANCE(MARKLIN_TRACK_TYPE_A, "D9", "E12", 1200)
TRACK_RESISTANCE(MARKLIN_TRACK_TYPE_A, "E12", "D11", 1200)
TRACK_RESISTANCE(MARKLIN_TRACK_TYPE_A, "D11", "C16", 1200)

// Track B resistance configurations
TRACK_RESISTANCE(MARKLIN_TRACK_TYPE_B, "MR1", "A9", 600)

#endif /* __MARKLIN_EDGE_RESISTANCE_LIST_H__ */
