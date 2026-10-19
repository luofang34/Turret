//! Confirmed manager primary ownership, independently of command writes.

use turret_core::{Error, Tick};

const STATUS_FRESHNESS_US: u64 = 1_000_000;

#[derive(Debug, Default)]
pub(super) struct Authority {
    client: Option<(u8, u8)>,
    primary: Option<((u8, u8), Tick)>,
    claim_sent: Option<Tick>,
}

impl Authority {
    pub(super) fn request_claim(&mut self, system_id: u8, component_id: u8) -> Result<(), Error> {
        if system_id == 0 || component_id == 0 {
            return Err(Error::InvalidValue);
        }
        let client = (system_id, component_id);
        if self.client == Some(client) {
            return Ok(());
        }
        if self.client.is_some() {
            return Err(Error::Unavailable);
        }
        self.client = Some(client);
        self.claim_sent = None;
        Ok(())
    }

    pub(super) fn observe_status(
        &mut self,
        system_id: u8,
        component_id: u8,
        now: Tick,
    ) -> Result<(), Error> {
        if let Some((_, previous)) = self.primary {
            now.elapsed_since(previous)?;
        }
        self.primary = Some(((system_id, component_id), now));
        Ok(())
    }

    pub(super) fn status_needed(&self, now: Tick) -> Result<bool, Error> {
        self.primary.map_or(Ok(true), |(_, at)| {
            now.elapsed_since(at).map(|age| age >= STATUS_FRESHNESS_US)
        })
    }

    pub(super) fn claim_due(&self, now: Tick) -> Result<bool, Error> {
        if self.client.is_none() || self.claim_sent.is_some() || self.status_needed(now)? {
            return Ok(false);
        }
        Ok(self.primary.is_some_and(|(owner, _)| owner == (0, 0)))
    }

    pub(super) fn has_authority(&self, now: Tick) -> Result<bool, Error> {
        if self.status_needed(now)? {
            return Ok(false);
        }
        Ok(self
            .client
            .is_some_and(|client| self.primary.is_some_and(|(owner, _)| owner == client)))
    }

    pub(super) fn record_claim_sent(&mut self, now: Tick) -> Result<(), Error> {
        if !self.claim_due(now)? {
            return Err(Error::Unavailable);
        }
        self.claim_sent = Some(now);
        Ok(())
    }
}
