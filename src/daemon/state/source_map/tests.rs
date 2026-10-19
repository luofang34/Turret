use super::*;

#[test]
fn evict_for_insert_drops_entries_past_ttl_keeps_fresh() {
    // The eviction helper runs against a synthetic map so the test
    // doesn't have to actually wait 5 minutes. Stamp two entries:
    // one ten minutes ago (well past the TTL) and one a moment ago.
    let mut map: HashMap<ControlSource, Instant> = HashMap::new();
    let now = Instant::now();
    let stale = now - Duration::from_secs(600);
    let fresh = now - Duration::from_millis(100);
    map.insert(ControlSource::Mavlink(1), stale);
    map.insert(ControlSource::UnixSocket(99), fresh);
    assert_eq!(map.len(), 2);

    evict_for_insert(&mut map, now);
    assert_eq!(map.len(), 1);
    assert!(map.contains_key(&ControlSource::UnixSocket(99)));
    assert!(!map.contains_key(&ControlSource::Mavlink(1)));
}

#[test]
fn evict_for_insert_enforces_hard_cap_under_burst() {
    // Synthesise a burst that the TTL cannot help with: every entry
    // is fresh (within the 5-min window), but the count exceeds the
    // hard cap. After the helper runs the map must be exactly at
    // `MAX_TRACKED_SOURCES - 1` so the caller's insert leaves it at
    // MAX. The single evicted entry is the oldest one we put in.
    let mut map: HashMap<ControlSource, Instant> = HashMap::new();
    let now = Instant::now();

    // Fill to exactly the cap with timestamps that order
    // monotonically (entry `i` is `i` ms before `now`, so larger `i`
    // means older).
    for i in 0..MAX_TRACKED_SOURCES as u32 {
        map.insert(
            ControlSource::UnixSocket(i),
            now - Duration::from_millis(u64::from(i)),
        );
    }
    assert_eq!(map.len(), MAX_TRACKED_SOURCES);

    evict_for_insert(&mut map, now);

    // Hard cap kicked in: one entry dropped, room for the imminent insert.
    assert_eq!(map.len(), MAX_TRACKED_SOURCES - 1);
    // The evicted entry is the oldest (highest `i`).
    let evicted = ControlSource::UnixSocket(MAX_TRACKED_SOURCES as u32 - 1);
    assert!(!map.contains_key(&evicted));
}
