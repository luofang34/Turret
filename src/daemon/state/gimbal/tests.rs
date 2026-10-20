use super::*;
use crate::daemon::attitude::MAX_MEASUREMENT_AGE;
use crate::gimbal::DeviceObservation;
use std::time::Duration;
use turret_core::{Angle, Axes, ObservationSource, ReferenceFrame};

fn sample(sampled_at: Instant) -> DeviceObservation {
    DeviceObservation {
        epoch: 0,
        angles: Axes {
            pitch: Some(Angle::ZERO),
            roll: None,
            yaw: None,
        },
        frame: ReferenceFrame::Vehicle,
        zoom: None,
        source: ObservationSource::DeviceMeasured,
        sampled_at: Some(sampled_at),
        received_at: sampled_at,
    }
}

#[test]
fn invalid_or_repeated_feedback_cannot_refresh_cached_measurements() {
    let state = GimbalStateStore::new();
    let now = Instant::now();
    let first = sample(now - Duration::from_millis(20));
    assert!(state.update_observation(&first));
    let cached = state.snapshot_cached();
    let newer = sample(now);
    let invalid = [
        first,
        sample(now - Duration::from_secs(2)),
        DeviceObservation {
            sampled_at: None,
            ..newer
        },
        DeviceObservation {
            source: ObservationSource::Estimated,
            ..newer
        },
        DeviceObservation {
            source: ObservationSource::OperatorAssumed,
            ..newer
        },
        DeviceObservation {
            frame: ReferenceFrame::Mount,
            ..newer
        },
        DeviceObservation { epoch: 1, ..newer },
        DeviceObservation {
            received_at: now + Duration::from_secs(1),
            ..newer
        },
        DeviceObservation {
            sampled_at: Some(now + Duration::from_secs(1)),
            ..newer
        },
    ];
    for invalid in invalid {
        assert!(!state.update_observation(&invalid), "{invalid:?}");
        let retained = state.snapshot_cached();
        assert_eq!(retained.measured_at, cached.measured_at);
        assert_eq!(retained.pitch, cached.pitch);
        assert_eq!(retained.roll, None);
    }
    assert!(state.update_observation(&newer));
    assert!(state.snapshot_cached().measured_at > cached.measured_at);
}

#[test]
fn epoch_change_clears_values_and_the_sample_ordering_baseline() {
    let state = GimbalStateStore::new();
    let now = Instant::now();
    assert!(state.update_observation(&sample(now)));
    state.invalidate(1);
    assert!(state.snapshot_cached().measured_at.is_none());
    assert!(state.snapshot_cached().pitch.is_none());
    assert!(!state.update_observation(&sample(Instant::now())));
    let next = DeviceObservation {
        epoch: 1,
        source: ObservationSource::TransformedMeasurement,
        ..sample(now - Duration::from_millis(1))
    };
    assert!(state.update_observation(&next));
    assert_eq!(state.snapshot_cached().connection_epoch, 1);
}

#[test]
fn freshness_uses_acquisition_time_and_rejects_inconsistent_clocks() {
    let now = Instant::now();
    let acquired = now - MAX_MEASUREMENT_AGE;
    let valid = DeviceObservation {
        received_at: now,
        ..sample(acquired)
    };
    assert_eq!(
        valid.measurement_time(now, MAX_MEASUREMENT_AGE),
        Some(acquired)
    );
    assert_eq!(
        valid.measurement_time(now + Duration::from_nanos(1), MAX_MEASUREMENT_AGE),
        None
    );
    let invalid = DeviceObservation {
        received_at: acquired - Duration::from_nanos(1),
        ..valid
    };
    assert_eq!(invalid.measurement_time(now, MAX_MEASUREMENT_AGE), None);
}
