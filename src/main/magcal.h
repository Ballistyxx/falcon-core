/*
 * Magnetometer hard- and soft-iron calibration.
 *
 * The correction is the standard per-axis affine model:
 *     calibrated[i] = (raw[i] - offset[i]) * scale[i]
 *   - offset[]  removes the hard-iron bias (the center of the sampled sphere)
 *   - scale[]   removes the diagonal soft-iron distortion (per-axis gain)
 *
 * The calibration is computed on the ground-station dashboard from a sample
 * sweep, pushed to the device over the WebSocket ("mag_cal" command), and
 * persisted in NVS so it survives reboots. Heading uses the calibrated vector;
 * the raw magnetometer is still what gets streamed in telemetry so the
 * dashboard can always re-derive calibration from uncorrected data.
 */

#ifndef FALCON_MAGCAL_H_
#define FALCON_MAGCAL_H_

typedef struct {
    float offset[3];   /* hard-iron center, raw mag units (uT) */
    float scale[3];    /* soft-iron per-axis gain (~1.0) */
} magcal_t;

/* Load stored calibration from NVS (identity if none). Call once at boot,
 * after nvs_flash_init(). */
void magcal_init(void);

/* Copy out the active calibration. */
void magcal_get(magcal_t *out);

/* Replace the active calibration and persist it to NVS. */
void magcal_set(const magcal_t *cal);

/* Apply the calibration: out[i] = (raw[i] - offset[i]) * scale[i].
 * raw and out may alias. */
void magcal_apply(const float raw[3], float out[3]);

#endif /* FALCON_MAGCAL_H_ */
