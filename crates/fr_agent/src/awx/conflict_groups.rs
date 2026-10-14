//! AWX compact lane/crossing/site/reach/exit conflict relation, fixed source caad45cd (MIT).
use super::escape_moves::{rounded, Direction, Move};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
pub const TOL: f64 = 0.16;
pub const TOUCH: f64 = 1e-6;
pub const EXIT_TOL: f64 = 0.16;
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Serialize, Deserialize)]
pub struct Member {
    pub ball: String,
    pub index: usize,
}
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Serialize, Deserialize)]
pub enum Tag {
    Lane(String),
    Crossing(String),
    Exit(String),
    Site,
    Reach,
}
#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum GroupKey {
    Clique(BTreeSet<Member>),
    Biclique(BTreeSet<Member>, BTreeSet<Member>),
}
#[derive(Clone, Debug, Default)]
pub struct ConflictGroups {
    pub cliques: BTreeSet<BTreeSet<Member>>,
    pub bicliques: BTreeSet<(BTreeSet<Member>, BTreeSet<Member>)>,
    pub pairs: BTreeSet<(Member, Member)>,
    pub tags: BTreeMap<GroupKey, BTreeSet<Tag>>,
}
#[derive(Clone, Debug)]
pub struct LaneSpan {
    pub axis: u8,
    pub layer: String,
    pub coordinate: f64,
    pub start: f64,
    pub end: f64,
}
fn plain_span(m: &Move) -> LaneSpan {
    let source = m.site.unwrap_or(m.legs[0].a);
    let axis = u8::from(matches!(m.direction, Direction::Up | Direction::Down));
    let along = usize::from(axis);
    let coordinate = m.exit_pt[1 - along];
    LaneSpan {
        axis,
        layer: m.layer.clone(),
        coordinate: rounded(coordinate, 3),
        start: source[along].min(m.exit_pt[along]),
        end: source[along].max(m.exit_pt[along]),
    }
}
pub fn lane_spans(m: &Move) -> Vec<LaneSpan> {
    if m.legs.is_empty() && m.site.is_none() {
        return vec![];
    }
    if m.climb == 0 && !m.street {
        return vec![plain_span(m)];
    }
    let mut out = Vec::new();
    for leg in &m.legs {
        if leg.layer != m.layer && !m.street {
            continue;
        }
        let dx = (leg.a[0] - leg.b[0]).abs();
        let dy = (leg.a[1] - leg.b[1]).abs();
        if dx < 1e-6 && dy > 1e-6 {
            out.push(LaneSpan {
                axis: 1,
                layer: leg.layer.clone(),
                coordinate: rounded(leg.a[0], 3),
                start: leg.a[1].min(leg.b[1]),
                end: leg.a[1].max(leg.b[1]),
            });
        } else if dy < 1e-6 && dx > 1e-6 {
            out.push(LaneSpan {
                axis: 0,
                layer: leg.layer.clone(),
                coordinate: rounded(leg.a[1], 3),
                start: leg.a[0].min(leg.b[0]),
                end: leg.a[0].max(leg.b[0]),
            });
        }
    }
    if out.is_empty() {
        vec![plain_span(m)]
    } else {
        out
    }
}
pub fn site_key(m: &Move) -> Option<(i64, i64)> {
    m.site.map(|p| {
        (
            (rounded(p[0], 3) * 1000.0).round_ties_even() as i64,
            (rounded(p[1], 3) * 1000.0).round_ties_even() as i64,
        )
    })
}
fn windows(values: &[f64], tol: f64) -> Vec<Vec<f64>> {
    let mut vs = values.to_vec();
    vs.sort_by(f64::total_cmp);
    vs.dedup();
    let mut out = Vec::new();
    let mut last: Option<usize> = None;
    let mut j = 0;
    for i in 0..vs.len() {
        j = j.max(i);
        while j + 1 < vs.len() && vs[j + 1] - vs[i] < tol {
            j += 1;
        }
        if last.is_none_or(|last| j > last) {
            out.push(vs[i..=j].to_vec());
            last = Some(j);
        }
    }
    out
}
fn interval_cliques(items: Vec<(f64, f64, Member)>) -> Vec<BTreeSet<Member>> {
    let mut events = Vec::new();
    for (a, b, member) in items {
        events.push((a, 1, member.clone()));
        events.push((b + TOUCH, 0, member));
    }
    events.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
    let mut live = BTreeSet::new();
    let mut out = Vec::new();
    let mut grown = false;
    for (_, kind, member) in events {
        if kind == 1 {
            live.insert(member);
            grown = true;
        } else {
            if grown && live.len() > 1 {
                out.push(live.clone());
            }
            grown = false;
            live.remove(&member);
        }
    }
    out
}
impl ConflictGroups {
    fn clique(&mut self, g: BTreeSet<Member>, tag: Tag) {
        self.cliques.insert(g.clone());
        self.tags
            .entry(GroupKey::Clique(g))
            .or_default()
            .insert(tag);
    }
    fn biclique(&mut self, a: BTreeSet<Member>, b: BTreeSet<Member>, tag: Tag) {
        if a.is_empty() || b.is_empty() || a.union(&b).count() <= 1 {
            return;
        }
        let (a, b) = if a <= b { (a, b) } else { (b, a) };
        self.bicliques.insert((a.clone(), b.clone()));
        self.tags
            .entry(GroupKey::Biclique(a, b))
            .or_default()
            .insert(tag);
    }
}
#[derive(Clone)]
struct Span {
    lane: LaneSpan,
    member: Member,
    climbs: bool,
}
pub fn conflict_groups<F>(
    menu: &BTreeMap<String, Vec<Move>>,
    stack: bool,
    stack_pitch: f64,
    via_radius: F,
    reach_extra: f64,
    xing: u32,
) -> ConflictGroups
where
    F: Fn(&str, usize, &Move) -> f64,
{
    let moves: Vec<_> = menu
        .iter()
        .flat_map(|(key, moves)| {
            moves.iter().enumerate().map(move |(i, m)| {
                (
                    Member {
                        ball: key.clone(),
                        index: i,
                    },
                    m,
                )
            })
        })
        .collect();
    let spans: Vec<_> = moves
        .iter()
        .flat_map(|(member, m)| {
            lane_spans(m).into_iter().map(move |lane| Span {
                lane,
                member: member.clone(),
                climbs: m.climb > 0 || m.street,
            })
        })
        .collect();
    let mut out = ConflictGroups::default();
    let mut by_axis: BTreeMap<(u8, String), Vec<&Span>> = BTreeMap::new();
    for s in &spans {
        by_axis
            .entry((s.lane.axis, s.lane.layer.clone()))
            .or_default()
            .push(s);
    }
    for ((_, layer), ss) in &by_axis {
        for window in windows(
            &ss.iter().map(|s| s.lane.coordinate).collect::<Vec<_>>(),
            TOL,
        ) {
            let items = ss
                .iter()
                .filter(|s| window.contains(&s.lane.coordinate))
                .map(|s| (s.lane.start, s.lane.end, s.member.clone()))
                .collect();
            for g in interval_cliques(items) {
                out.clique(g, Tag::Lane(layer.clone()));
            }
        }
    }
    if xing > 0 {
        let layers: BTreeSet<_> = spans.iter().map(|s| s.lane.layer.clone()).collect();
        for layer in layers {
            let rows = windows(
                &spans
                    .iter()
                    .filter(|s| s.lane.axis == 0 && s.lane.layer == layer)
                    .map(|s| s.lane.coordinate)
                    .collect::<Vec<_>>(),
                0.0,
            );
            let mut row_coords: Vec<_> = rows.into_iter().flatten().collect();
            row_coords.sort_by(f64::total_cmp);
            row_coords.dedup();
            let mut col_coords: Vec<_> = spans
                .iter()
                .filter(|s| s.lane.axis == 1 && s.lane.layer == layer)
                .map(|s| s.lane.coordinate)
                .collect();
            col_coords.sort_by(f64::total_cmp);
            col_coords.dedup();
            for y in row_coords {
                for &x in &col_coords {
                    let rows: Vec<_> = spans
                        .iter()
                        .filter(|s| {
                            s.lane.layer == layer
                                && s.lane.axis == 0
                                && s.lane.coordinate == y
                                && s.lane.start - TOL < x
                                && x < s.lane.end + TOL
                        })
                        .collect();
                    if rows.is_empty() {
                        continue;
                    }
                    let cols: Vec<_> = spans
                        .iter()
                        .filter(|s| {
                            s.lane.layer == layer
                                && s.lane.axis == 1
                                && s.lane.coordinate == x
                                && s.lane.start - TOL < y
                                && y < s.lane.end + TOL
                        })
                        .collect();
                    if cols.is_empty() {
                        continue;
                    }
                    let r = rows.iter().map(|s| s.member.clone()).collect();
                    let c = cols.iter().map(|s| s.member.clone()).collect();
                    if xing >= 2 {
                        out.biclique(r, c, Tag::Crossing(layer.clone()));
                    } else {
                        out.biclique(
                            r,
                            cols.iter()
                                .filter(|s| s.climbs)
                                .map(|s| s.member.clone())
                                .collect(),
                            Tag::Crossing(layer.clone()),
                        );
                        out.biclique(
                            rows.iter()
                                .filter(|s| s.climbs)
                                .map(|s| s.member.clone())
                                .collect(),
                            c,
                            Tag::Crossing(layer.clone()),
                        );
                    }
                }
            }
        }
    }
    let mut by_site: BTreeMap<(i64, i64), Vec<(&Member, &Move)>> = BTreeMap::new();
    for (member, m) in &moves {
        if let Some(site) = site_key(m) {
            by_site.entry(site).or_default().push((member, m));
        }
    }
    for site_moves in by_site.values() {
        if site_moves.len() > 1 {
            out.clique(
                site_moves
                    .iter()
                    .map(|(member, _)| (*member).clone())
                    .collect(),
                Tag::Site,
            );
        }
    }
    let mut by_coord: BTreeMap<(u8, i64), Vec<&Span>> = BTreeMap::new();
    for s in &spans {
        by_coord
            .entry((
                s.lane.axis,
                (s.lane.coordinate * 1000.0).round_ties_even() as i64,
            ))
            .or_default()
            .push(s);
    }
    for site_moves in by_site.values() {
        for ((axis, _), ss) in &by_coord {
            let mut reached: BTreeMap<BTreeSet<Member>, BTreeSet<Member>> = BTreeMap::new();
            for s in ss {
                let mut who = BTreeSet::new();
                for (member, m) in site_moves {
                    let site = m.site.unwrap();
                    let r = via_radius(&member.ball, member.index, m) + reach_extra;
                    let (perp, along) = if *axis == 0 {
                        (site[1], site[0])
                    } else {
                        (site[0], site[1])
                    };
                    if (perp - s.lane.coordinate).abs() < r
                        && s.lane.start - r < along
                        && along < s.lane.end + r
                    {
                        who.insert((*member).clone());
                    }
                }
                if !who.is_empty() {
                    reached.entry(who).or_default().insert(s.member.clone());
                }
            }
            for (who, runs) in reached {
                out.biclique(who, runs, Tag::Reach);
            }
        }
    }
    let mut by_face: BTreeMap<(Option<String>, Direction), Vec<(f64, Member)>> = BTreeMap::new();
    for (member, m) in moves {
        by_face
            .entry((stack.then(|| m.layer.clone()), m.direction))
            .or_default()
            .push((m.exit_pt[m.direction.along_axis()], member));
    }
    let radius = if stack {
        stack_pitch.max(EXIT_TOL)
    } else {
        EXIT_TOL
    };
    for ((layer, _), points) in by_face {
        for g in interval_cliques(
            points
                .into_iter()
                .map(|(v, member)| (v - radius / 2.0, v + radius / 2.0 - TOUCH, member))
                .collect(),
        ) {
            out.clique(g, Tag::Exit(layer.clone().unwrap_or_default()));
        }
    }
    out
}
pub fn expand(groups: &ConflictGroups) -> BTreeSet<(Member, Member)> {
    let mut out = groups.pairs.clone();
    let mut pair = |a: &Member, b: &Member| {
        if a.ball != b.ball {
            out.insert(if a <= b {
                (a.clone(), b.clone())
            } else {
                (b.clone(), a.clone())
            });
        }
    };
    for g in &groups.cliques {
        let members: Vec<_> = g.iter().collect();
        for i in 0..members.len() {
            for j in i + 1..members.len() {
                pair(members[i], members[j]);
            }
        }
    }
    for (a, b) in &groups.bicliques {
        for x in a {
            for y in b {
                pair(x, y);
            }
        }
    }
    out
}
