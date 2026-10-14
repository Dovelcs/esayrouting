//! Equivalent geometry of AWX escape_moves.py at caad45cd252f9a50f1fb4b3b15461bdf1b9457a1 (MIT).
use serde::{Deserialize, Serialize};
pub type Point = [f64; 2];
pub const RUN: &str = "*run*";
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Pad {
    pub key: String,
    pub net: String,
    pub net_id: i32,
    pub component_ref: String,
    pub pad_number: String,
    pub point: Point,
    pub layers: Vec<String>,
    pub size: Point,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Grid {
    pub xs: Vec<f64>,
    pub ys: Vec<f64>,
    pub pitch_x: f64,
    pub pitch_y: f64,
    pub bbox: [f64; 4],
}
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Direction {
    Left,
    Right,
    Up,
    Down,
}
impl Direction {
    pub const ALL: [Self; 4] = [Self::Left, Self::Right, Self::Up, Self::Down];
    pub fn delta(self) -> (i32, i32) {
        match self {
            Self::Left => (-1, 0),
            Self::Right => (1, 0),
            Self::Up => (0, -1),
            Self::Down => (0, 1),
        }
    }
    pub fn along_axis(self) -> usize {
        if matches!(self, Self::Up | Self::Down) {
            0
        } else {
            1
        }
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MoveKind {
    Surface,
    Dogbone,
    ViaInPad,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Leg {
    pub a: Point,
    pub b: Point,
    pub layer: String,
}
impl Leg {
    pub fn new(a: Point, b: Point, layer: &str) -> Self {
        Self {
            a,
            b,
            layer: layer.into(),
        }
    }
    pub fn length(&self) -> f64 {
        distance(self.a, self.b)
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Move {
    pub net: String,
    pub kind: MoveKind,
    pub direction: Direction,
    pub layer: String,
    pub exit_pt: Point,
    pub vias: u32,
    pub legs: Vec<Leg>,
    pub site: Option<Point>,
    pub climb: u32,
    pub street: bool,
    #[serde(default)]
    pub runs_on: Vec<String>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct EscapeOptions {
    pub margin: f64,
    pub climb: u32,
    pub own_line: bool,
    pub straight: bool,
    pub street: u32,
    pub street_pitch: f64,
    pub street_dirs: Option<Vec<Direction>>,
}
impl Default for EscapeOptions {
    fn default() -> Self {
        Self {
            margin: 0.0,
            climb: 0,
            own_line: false,
            straight: false,
            street: 0,
            street_pitch: 0.0,
            street_dirs: None,
        }
    }
}
pub fn short_name(name: &str) -> &str {
    name.rsplit('/').next().unwrap_or(name)
}
pub fn distance(a: Point, b: Point) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}
pub fn rounded(value: f64, digits: i32) -> f64 {
    super::numeric::round_digits(value, digits)
        .expect("AWX coordinates validated finite before rounding")
}
pub fn grid_of(pads: &[Pad]) -> Result<Grid, String> {
    if pads.is_empty() {
        return Err("array has no pads".into());
    }
    let coords = |axis: usize| {
        let mut xs: Vec<_> = pads.iter().map(|p| rounded(p.point[axis], 3)).collect();
        xs.sort_by(f64::total_cmp);
        xs.dedup();
        xs
    };
    let xs = coords(0);
    let ys = coords(1);
    let pitch = |v: &[f64]| {
        v.windows(2)
            .map(|w| w[1] - w[0])
            .reduce(f64::min)
            .unwrap_or(0.0)
    };
    let bbox = [
        pads.iter().map(|p| p.point[0]).reduce(f64::min).unwrap(),
        pads.iter().map(|p| p.point[1]).reduce(f64::min).unwrap(),
        pads.iter().map(|p| p.point[0]).reduce(f64::max).unwrap(),
        pads.iter().map(|p| p.point[1]).reduce(f64::max).unwrap(),
    ];
    Ok(Grid {
        pitch_x: pitch(&xs),
        pitch_y: pitch(&ys),
        xs,
        ys,
        bbox,
    })
}
fn movement(
    pad: &Pad,
    kind: MoveKind,
    direction: Direction,
    layer: &str,
    exit_pt: Point,
    legs: Vec<Leg>,
    site: Option<Point>,
    climb: u32,
    street: bool,
) -> Move {
    Move {
        net: short_name(&pad.net).into(),
        kind,
        direction,
        layer: layer.into(),
        exit_pt,
        vias: u32::from(site.is_some()),
        legs,
        site,
        climb,
        street,
        runs_on: vec![],
    }
}
fn empty_bands(values: &[f64], lo: f64, hi: f64, pitch: f64) -> Vec<(f64, f64)> {
    if pitch <= 0.0 {
        return vec![];
    }
    let count = ((hi - lo) / pitch).round_ties_even() as usize;
    let lattice: Vec<_> = (0..=count).map(|k| lo + k as f64 * pitch).collect();
    let has = |v: f64| values.iter().any(|w| (v - w).abs() < pitch / 4.0);
    let mut out = Vec::new();
    let mut k = 0;
    while k < lattice.len() {
        if has(lattice[k]) {
            k += 1;
            continue;
        }
        let mut j = k;
        while j + 1 < lattice.len() && !has(lattice[j + 1]) {
            j += 1;
        }
        if k > 0 && j + 1 < lattice.len() {
            out.push((lattice[k - 1], lattice[j + 1]));
        }
        k = j + 1;
    }
    out
}

pub fn enumerate_moves<C, V>(
    pad: &Pad,
    grid: &Grid,
    layers: &[String],
    options: &EscapeOptions,
    mut clear: C,
    mut via_clear: V,
) -> Vec<Move>
where
    C: FnMut(Point, Point, &str) -> bool,
    V: FnMut(Point, &str) -> bool,
{
    if layers.is_empty() {
        return vec![];
    }
    let point = pad.point;
    let (px, py) = (point[0], point[1]);
    let home = layers
        .iter()
        .find(|l| pad.layers.contains(l))
        .unwrap_or(&layers[0]);
    let others: Vec<_> = layers.iter().filter(|l| *l != home).collect();
    let [x0, y0, x1, y1] = grid.bbox;
    let (hx, hy) = (grid.pitch_x / 2.0, grid.pitch_y / 2.0);
    let mut out = Vec::new();
    let edge = |direction: Direction| {
        let (dx, dy) = direction.delta();
        if dx != 0 {
            [
                if dx < 0 {
                    x0 - hx - options.margin
                } else {
                    x1 + hx + options.margin
                },
                py,
            ]
        } else {
            [
                px,
                if dy < 0 {
                    y0 - hy - options.margin
                } else {
                    y1 + hy + options.margin
                },
            ]
        }
    };
    for direction in Direction::ALL {
        let (dx, _) = direction.delta();
        for sign in [-1.0, 1.0] {
            let (gate, e) = if dx != 0 {
                let gate = [px, py + sign * hy];
                if !(y0 < gate[1] && gate[1] < y1) {
                    continue;
                }
                (gate, [edge(direction)[0], gate[1]])
            } else {
                let gate = [px + sign * hx, py];
                if !(x0 < gate[0] && gate[0] < x1) {
                    continue;
                }
                (gate, [gate[0], edge(direction)[1]])
            };
            if clear(point, gate, home) && clear(gate, e, home) {
                out.push(movement(
                    pad,
                    MoveKind::Surface,
                    direction,
                    home,
                    e,
                    vec![Leg::new(point, gate, home), Leg::new(gate, e, home)],
                    None,
                    0,
                    false,
                ));
            }
        }
    }
    if options.straight {
        for direction in Direction::ALL {
            let e = edge(direction);
            if clear(point, e, home) {
                out.push(movement(
                    pad,
                    MoveKind::Surface,
                    direction,
                    home,
                    e,
                    vec![Leg::new(point, e, home)],
                    None,
                    0,
                    false,
                ));
            }
        }
    }
    for layer in &others {
        if !layers.iter().all(|l| via_clear(point, l)) {
            break;
        }
        for direction in Direction::ALL {
            let e = edge(direction);
            if clear(point, e, layer) {
                out.push(movement(
                    pad,
                    MoveKind::ViaInPad,
                    direction,
                    layer,
                    e,
                    vec![Leg::new(point, e, layer)],
                    Some(point),
                    0,
                    false,
                ));
            }
        }
    }
    for (sx, sy) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
        let site = [px + sx * hx, py + sy * hy];
        if !(x0 < site[0] && site[0] < x1 && y0 < site[1] && site[1] < y1)
            || !layers.iter().all(|l| via_clear(site, l))
            || !clear(point, site, home)
        {
            continue;
        }
        for direction in Direction::ALL {
            let mut e = edge(direction);
            if direction.delta().0 != 0 {
                e[1] = site[1];
            } else {
                e[0] = site[0];
            }
            for layer in &others {
                if clear(site, e, layer) {
                    out.push(movement(
                        pad,
                        MoveKind::Dogbone,
                        direction,
                        layer,
                        e,
                        vec![Leg::new(point, site, home), Leg::new(site, e, layer)],
                        Some(site),
                        0,
                        false,
                    ));
                }
            }
        }
    }
    if options.climb > 0 {
        let mut starts = Vec::new();
        for (sx, sy) in [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)] {
            let site = [px + sx * hx, py + sy * hy];
            if x0 < site[0]
                && site[0] < x1
                && y0 < site[1]
                && site[1] < y1
                && layers.iter().all(|l| via_clear(site, l))
                && clear(point, site, home)
            {
                starts.push((MoveKind::Dogbone, site, vec![Leg::new(point, site, home)]));
            }
        }
        if layers.iter().all(|l| via_clear(point, l)) {
            starts.push((MoveKind::ViaInPad, point, vec![]));
        }
        for (kind, site, legs0) in starts {
            for layer in &others {
                for direction in Direction::ALL {
                    let (dx, _) = direction.delta();
                    let e0 = edge(direction);
                    let lanes: Vec<i32> = if options.own_line {
                        vec![0, -1, 1]
                    } else {
                        vec![-1, 1]
                    };
                    let gaps: Vec<(Point, Vec<Leg>)> = if kind == MoveKind::Dogbone {
                        vec![(site, vec![])]
                    } else if dx != 0 {
                        lanes
                            .iter()
                            .filter(|&&g| {
                                g == 0 || (x0 < px + g as f64 * hx && px + g as f64 * hx < x1)
                            })
                            .map(|&g| {
                                let q = [px + g as f64 * hx, py];
                                (
                                    q,
                                    if g == 0 {
                                        vec![]
                                    } else {
                                        vec![Leg::new(point, q, layer)]
                                    },
                                )
                            })
                            .collect()
                    } else {
                        lanes
                            .iter()
                            .filter(|&&g| {
                                g == 0 || (y0 < py + g as f64 * hy && py + g as f64 * hy < y1)
                            })
                            .map(|&g| {
                                let q = [px, py + g as f64 * hy];
                                (
                                    q,
                                    if g == 0 {
                                        vec![]
                                    } else {
                                        vec![Leg::new(point, q, layer)]
                                    },
                                )
                            })
                            .collect()
                    };
                    for (gap, legs_in) in gaps {
                        if legs_in.iter().any(|l| !clear(l.a, l.b, &l.layer)) {
                            continue;
                        }
                        for sign in [-1.0, 1.0] {
                            for h in 1..=2 * options.climb {
                                let (turn, e) = if dx != 0 {
                                    let ey = gap[1] + sign * h as f64 * hy;
                                    if !(y0 < ey && ey < y1) {
                                        break;
                                    }
                                    ([gap[0], ey], [e0[0], ey])
                                } else {
                                    let ex = gap[0] + sign * h as f64 * hx;
                                    if !(x0 < ex && ex < x1) {
                                        break;
                                    }
                                    ([ex, gap[1]], [ex, e0[1]])
                                };
                                if !clear(gap, turn, layer) {
                                    break;
                                }
                                if !clear(turn, e, layer) {
                                    continue;
                                }
                                let mut legs = legs0.clone();
                                legs.extend(legs_in.clone());
                                legs.push(Leg::new(gap, turn, layer));
                                legs.push(Leg::new(turn, e, layer));
                                out.push(movement(
                                    pad,
                                    kind,
                                    direction,
                                    layer,
                                    e,
                                    legs,
                                    Some(if kind == MoveKind::Dogbone {
                                        site
                                    } else {
                                        point
                                    }),
                                    (h + 1) / 2,
                                    false,
                                ));
                            }
                        }
                    }
                }
            }
        }
    }
    if options.street > 0 && options.street_pitch > 0.0 {
        for (axis, bands) in [
            (1, empty_bands(&grid.ys, y0, y1, grid.pitch_y)),
            (0, empty_bands(&grid.xs, x0, x1, grid.pitch_x)),
        ] {
            let half = if axis == 1 { hy } else { hx };
            let along_half = if axis == 1 { hx } else { hy };
            let pc = point[axis];
            let (lo_c, hi_c) = if axis == 1 { (x0, x1) } else { (y0, y1) };
            let pt = |along: f64, across: f64| {
                if axis == 1 {
                    [along, across]
                } else {
                    [across, along]
                }
            };
            for (a, b) in bands {
                let sign = if pc <= a + 1e-6 {
                    1.0
                } else if pc >= b - 1e-6 {
                    -1.0
                } else {
                    continue;
                };
                let (n1, n2) = (a + half, b - half);
                let lane_count = ((n2 - n1 - 2.0 * options.street_pitch) / options.street_pitch
                    + 1e-9)
                    .trunc() as i64
                    + 1;
                let mid = (n1 + n2) / 2.0;
                for i in 0..lane_count.max(0) {
                    let v = mid + (i as f64 - (lane_count - 1) as f64 / 2.0) * options.street_pitch;
                    if (v - mid) * sign > options.street_pitch / 2.0 {
                        continue;
                    }
                    for g in [0, -1, 1] {
                        let c = point[1 - axis] + g as f64 * along_half;
                        let legs_f = if g != 0 {
                            let gate = pt(c, pc + sign * half);
                            vec![Leg::new(point, gate, home), Leg::new(gate, pt(c, v), home)]
                        } else {
                            vec![Leg::new(point, pt(c, v), home)]
                        };
                        if !legs_f.iter().all(|l| clear(l.a, l.b, &l.layer)) {
                            continue;
                        }
                        let dirs = if axis == 1 {
                            [Direction::Left, Direction::Right]
                        } else {
                            [Direction::Up, Direction::Down]
                        };
                        for direction in dirs {
                            if options
                                .street_dirs
                                .as_ref()
                                .is_some_and(|dirs| !dirs.contains(&direction))
                            {
                                continue;
                            }
                            let (dx, dy) = direction.delta();
                            let dd = if axis == 1 { dx } else { dy } as f64;
                            let e0 = edge(direction);
                            for t in 0..options.street {
                                let along = c + dd * t as f64 * along_half;
                                if !(lo_c < along && along < hi_c) {
                                    break;
                                }
                                let site = pt(along, v);
                                let mut legs = legs_f.clone();
                                if t > 0 {
                                    legs.push(Leg::new(pt(c, v), site, home));
                                    if !clear(pt(c, v), site, home) {
                                        break;
                                    }
                                }
                                if !layers.iter().all(|l| via_clear(site, l)) {
                                    continue;
                                }
                                let e = if axis == 1 {
                                    [e0[0], site[1]]
                                } else {
                                    [site[0], e0[1]]
                                };
                                for layer in &others {
                                    if clear(site, e, layer) {
                                        let mut all = legs.clone();
                                        all.push(Leg::new(site, e, layer));
                                        out.push(movement(
                                            pad,
                                            MoveKind::Dogbone,
                                            direction,
                                            layer,
                                            e,
                                            all,
                                            Some(site),
                                            0,
                                            true,
                                        ));
                                    }
                                }
                            }
                        }
                        break;
                    }
                }
            }
        }
    }
    out
}

pub fn dedupe_climbs(moves: Vec<Move>) -> Vec<Move> {
    use std::collections::BTreeMap;
    let mut best: BTreeMap<(MoveKind, Direction, String, i64), (u32, f64, usize)> = BTreeMap::new();
    for (i, m) in moves.iter().enumerate() {
        if m.climb == 0 {
            continue;
        }
        let key = (
            m.kind,
            m.direction,
            m.layer.clone(),
            (rounded(m.exit_pt[m.direction.along_axis()], 2) * 100.0).round_ties_even() as i64,
        );
        let cost = (m.vias, m.legs.iter().map(Leg::length).sum::<f64>(), i);
        if best
            .get(&key)
            .is_none_or(|old| cost.0 < old.0 || (cost.0 == old.0 && cost.1 < old.1))
        {
            best.insert(key, cost);
        }
    }
    let keep: std::collections::BTreeSet<_> = best.values().map(|c| c.2).collect();
    moves
        .into_iter()
        .enumerate()
        .filter(|(i, m)| m.climb == 0 || keep.contains(i))
        .map(|(_, m)| m)
        .collect()
}
