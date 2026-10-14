//! Selection state for `route_bus.py`'s coherent-net K prefix.
//!
//! The source owns ladder construction and ordering: coherent rivers, largest
//! first, then singleton nets, with nets lacking a free source stub omitted.
//! This module deliberately consumes that already-filtered order; it does not
//! infer rivers or duplicate the AWX planner. `--k` selects a prefix of this
//! order. It is independent of `fab_rules::FabFloor` escalation.

/// The reason a requested bus net is absent from the selected prefix.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LadderOmission {
    /// The source fanout did not produce a coherent ladder entry for this net.
    NotInLadder,
    /// The net is in the ladder, but beyond the selected `--k` prefix.
    BeyondK,
}

/// End condition exposed by the selector's pull interface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LadderEnd {
    /// Every selected item has been yielded.
    PrefixExhausted,
    /// The selected prefix contains no nets.
    EmptyPrefix,
}

/// Ordered, bounded view of the route_bus coherent ladder.
///
/// `ordered_nets` must be the source-produced order after its free-stub filter.
/// `requested_k` follows Python's original behavior: absent or zero means the
/// full ladder, positive values clamp to its length, and negative values use
/// Python prefix-slice semantics (exclude that many trailing entries).
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RouteLadder {
    ordered_nets: Vec<String>,
    selected_len: usize,
    cursor: usize,
    rounds: usize,
}

impl RouteLadder {
    pub fn new(ordered_nets: Vec<String>, requested_k: Option<i64>, rounds: usize) -> Self {
        let len = ordered_nets.len();
        let selected_len = match requested_k {
            None | Some(0) => len,
            Some(k) if k > 0 => (k as u64).min(len as u64) as usize,
            Some(k) => len.saturating_sub(k.unsigned_abs().min(len as u64) as usize),
        };
        Self {
            ordered_nets,
            selected_len,
            cursor: 0,
            rounds,
        }
    }

    /// The effective K passed through the original route_bus/whole_route seam.
    pub fn k(&self) -> usize {
        self.selected_len
    }

    /// The whole-route round count. It is not a ladder rung or a per-net retry.
    pub fn rounds(&self) -> usize {
        self.rounds
    }

    /// The original source-ordered candidate ladder, including entries beyond K.
    pub fn ordered_nets(&self) -> &[String] {
        &self.ordered_nets
    }

    /// The selected route_bus prefix, in its original order.
    pub fn selected_nets(&self) -> &[String] {
        &self.ordered_nets[..self.selected_len]
    }

    /// Pull the next selected net. `None` means the explicit terminal state
    /// returned by [`Self::termination`].
    pub fn next_item(&mut self) -> Option<&str> {
        if self.cursor >= self.selected_len {
            return None;
        }
        let item = self.ordered_nets[self.cursor].as_str();
        self.cursor += 1;
        Some(item)
    }

    pub fn termination(&self) -> LadderEnd {
        if self.selected_len == 0 {
            LadderEnd::EmptyPrefix
        } else {
            LadderEnd::PrefixExhausted
        }
    }

    /// Explain why a bus candidate did not enter the selected route set.
    pub fn omission(&self, net: &str) -> Option<LadderOmission> {
        if self
            .selected_nets()
            .iter()
            .any(|candidate| candidate == net)
        {
            None
        } else if self.ordered_nets[self.selected_len..]
            .iter()
            .any(|candidate| candidate == net)
        {
            Some(LadderOmission::BeyondK)
        } else {
            Some(LadderOmission::NotInLadder)
        }
    }
}
