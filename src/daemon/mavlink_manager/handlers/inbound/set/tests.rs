use super::*;

#[test]
fn manual_rate_full_stick_maps_to_full_scale_deg_per_s() {
    assert_eq!(manual_rate_to_deg_per_s(1.0), MANUAL_RATE_FULL_DEG_S);
    assert_eq!(manual_rate_to_deg_per_s(-1.0), -MANUAL_RATE_FULL_DEG_S);
}

#[test]
fn manual_rate_half_stick_maps_to_half_scale() {
    assert_eq!(manual_rate_to_deg_per_s(0.5), MANUAL_RATE_FULL_DEG_S * 0.5);
    assert_eq!(
        manual_rate_to_deg_per_s(-0.25),
        MANUAL_RATE_FULL_DEG_S * -0.25
    );
}

#[test]
fn manual_rate_zero_maps_to_zero() {
    assert_eq!(manual_rate_to_deg_per_s(0.0), 0.0);
}

#[test]
fn manual_rate_nan_preserves_omitted_axis() {
    // Per-axis NaN must contribute no rate so any_rate_active() can
    // decide whether the whole command is a no-op based on the
    // *other* axis.
    assert!(manual_rate_to_deg_per_s(f32::NAN).is_nan());
}

#[test]
fn manual_rate_passes_through_out_of_range_values() {
    // Spec says -1..=1, but a buggy GCS may exceed that. Don't
    // sanitize here — clamp_pry runs against the configured device
    // limits later, which is the right place to enforce hardware
    // bounds. Here, just preserve the linear scaling so debug logs
    // can show what the GCS actually sent.
    assert_eq!(manual_rate_to_deg_per_s(2.0), MANUAL_RATE_FULL_DEG_S * 2.0);
}
