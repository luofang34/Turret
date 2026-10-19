//! Cover the peer-map eviction. Same shape as
//! `daemon::state::tests::evict_for_insert_*` so a future maintainer
//! reading either one immediately recognizes the pattern.

use super::*;

fn addr(port: u16) -> SocketAddr {
    SocketAddr::from(([127, 0, 0, 1], port))
}

#[test]
fn record_peer_inserts_and_refreshes() {
    let mut peers = HashMap::new();
    let t0 = Instant::now();
    record_peer(&mut peers, addr(1000), t0);
    assert_eq!(peers.len(), 1);
    assert_eq!(peers[&addr(1000)], t0);

    // Re-record the same peer at a later time → entry overwritten,
    // map size unchanged.
    let t1 = t0 + Duration::from_secs(1);
    record_peer(&mut peers, addr(1000), t1);
    assert_eq!(peers.len(), 1);
    assert_eq!(peers[&addr(1000)], t1);
}

#[test]
fn record_peer_drops_ttl_expired_entries() {
    let mut peers = HashMap::new();
    let t0 = Instant::now();
    record_peer(&mut peers, addr(1000), t0);
    record_peer(&mut peers, addr(1001), t0);
    assert_eq!(peers.len(), 2);

    // Advance past the TTL and insert a fresh peer. The two old
    // entries should be swept on the way in.
    let t_late = t0 + PEER_TTL + Duration::from_secs(1);
    record_peer(&mut peers, addr(1002), t_late);
    assert_eq!(peers.len(), 1, "stale peers were not evicted");
    assert!(peers.contains_key(&addr(1002)));
}

#[test]
fn record_peer_respects_hard_cap_under_burst() {
    let mut peers = HashMap::new();
    let t0 = Instant::now();
    // Fill the map within the TTL window so the TTL pass evicts
    // nothing — this exercises the hard-cap path.
    for i in 0..MAX_PEERS as u16 {
        record_peer(
            &mut peers,
            addr(2000 + i),
            t0 + Duration::from_millis(i.into()),
        );
    }
    assert_eq!(peers.len(), MAX_PEERS);
    let oldest = addr(2000);
    assert!(peers.contains_key(&oldest));

    // One more insert. Cap holds, oldest goes.
    let t_new = t0 + Duration::from_millis(MAX_PEERS as u64);
    record_peer(&mut peers, addr(9999), t_new);
    assert_eq!(peers.len(), MAX_PEERS);
    assert!(
        !peers.contains_key(&oldest),
        "oldest should have been evicted"
    );
    assert!(peers.contains_key(&addr(9999)));
}

#[test]
fn record_peer_burst_stays_capped() {
    // Sanity stress: insert many more than MAX_PEERS distinct
    // addresses inside one TTL window. Final size must equal the
    // cap, not the number of inserts.
    let mut peers = HashMap::new();
    let t0 = Instant::now();
    let burst = MAX_PEERS * 5;
    for i in 0..burst as u16 {
        record_peer(
            &mut peers,
            addr(10_000 + i),
            t0 + Duration::from_millis(i.into()),
        );
    }
    assert_eq!(peers.len(), MAX_PEERS, "hard cap must hold under burst");
}
