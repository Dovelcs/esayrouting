use crate::{
    planner::{PlanCandidate, RouteLeg},
    spec::{ChannelRegion, NetRule, RouteSpec},
};
use fr_engine::{
    board::{connectivity::clear_connected_sets_cache, BasicBoard, ItemKey, ItemKind, JavaVariant, RoutingBoard, TreeKind},
    drc::clearance_violation::{clearance_violations, ClearanceViolation},
    ids::{FixedState, NetNo},
    library::BoardLibrary,
    rules::BoardRules,
    structure::{Communication, Components, LayerStructure},
};
use fr_geom::{IntPoint, Point, Polyline};
use fr_settings::RouterSettings;
use serde::Serialize;
use std::{
    cell::RefCell,
    collections::{BTreeMap, BTreeSet},
    sync::Arc,
};
thread_local! { static VALIDATION_BASELINE_CACHE: RefCell<Vec<ValidationBaselineMemo>> = const { RefCell::new(Vec::new()) }; }

struct ValidationBaselineWitness {
    item_epoch: u64,
    global_epoch: u64,
    tree_item_epoch: u64,
    rules: Arc<BoardRules>,
    library: Arc<BoardLibrary>,
    components: Arc<Components>,
    layer_structure: Arc<LayerStructure>,
    communication: Communication,
    bounding_box: fr_geom::IntBox,
    tree_kind: TreeKind,
    tree_clearance_class: i32,
    java_variant: JavaVariant,
    stitching_vias: bool,
    awx_same_net_via_pad_contact: bool,
    pad_area_contacts: bool,
}

impl ValidationBaselineWitness {
    fn new(adapter: &BoardAdapter) -> Self {
        let board = &adapter.board;
        let items = board.item_repository();
        let tree = board.default_tree();
        Self {
            item_epoch: items.epoch(),
            global_epoch: items.global_epoch(),
            tree_item_epoch: tree.item_epoch(),
            rules: Arc::clone(&board.rules),
            library: Arc::clone(&board.library),
            components: Arc::clone(&board.components),
            layer_structure: Arc::clone(&board.layer_structure),
            communication: board.communication.clone(),
            bounding_box: board.bounding_box,
            tree_kind: tree.kind,
            tree_clearance_class: tree.compensated_clearance_class_no,
            java_variant: board.java_variant,
            stitching_vias: board.stitching_vias,
            awx_same_net_via_pad_contact: adapter.awx_same_net_via_pad_contact,
            pad_area_contacts: board.pad_area_contacts(),
        }
    }

    fn matches(&self, adapter: &BoardAdapter) -> bool {
        let board = &adapter.board;
        let items = board.item_repository();
        let tree = board.default_tree();
        self.item_epoch == items.epoch()
            && self.global_epoch == items.global_epoch()
            && self.tree_item_epoch == tree.item_epoch()
            && Arc::ptr_eq(&self.rules, &board.rules)
            && Arc::ptr_eq(&self.library, &board.library)
            && Arc::ptr_eq(&self.components, &board.components)
            && Arc::ptr_eq(&self.layer_structure, &board.layer_structure)
            && self.communication == board.communication
            && self.bounding_box == board.bounding_box
            && self.tree_kind == tree.kind
            && self.tree_clearance_class == tree.compensated_clearance_class_no
            && self.java_variant == board.java_variant
            && self.stitching_vias == board.stitching_vias
            && self.awx_same_net_via_pad_contact == adapter.awx_same_net_via_pad_contact
            && self.pad_area_contacts == board.pad_area_contacts()
    }
}

#[derive(Default)]
struct ValidationBaselineMemo {
    witness: Option<ValidationBaselineWitness>,
    violations: Option<Arc<BTreeSet<(i32, i32, i32, u64, u64)>>>,
    connectivity: Option<Arc<BTreeMap<String, Vec<Vec<i32>>>>>,
    debug_strings: BTreeMap<i32, BaselineDebugStrings>,
}

#[derive(Default)]
struct BaselineDebugStrings {
    item_version: u64,
    kind: Option<Arc<str>>,
    tile_shapes: Option<Arc<str>>,
}

#[derive(Clone, Copy)]
enum BaselineDebugField {
    Kind,
    TileShapes,
}

pub(crate) fn clear_validation_baseline_cache() {
    VALIDATION_BASELINE_CACHE.with(|cache| cache.borrow_mut().clear());
    clear_connected_sets_cache();
}

fn cached_baseline_debug<T: std::fmt::Debug>(
    adapter: &BoardAdapter,
    key: ItemKey,
    field: BaselineDebugField,
    value: &T,
) -> Arc<str> {
    let board = &adapter.board;
    let item = board.item(key);
    let item_id = item.id().0;
    let item_version = board.item_repository().version(key);
    VALIDATION_BASELINE_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        let memo_index = cache.iter().position(|memo| {
            memo.witness
                .as_ref()
                .is_some_and(|witness| witness.matches(adapter))
        });
        let memo_index = memo_index.unwrap_or_else(|| {
            if cache.len() == 2 {
                cache.remove(0);
            }
            cache.push(ValidationBaselineMemo {
                witness: Some(ValidationBaselineWitness::new(adapter)),
                ..ValidationBaselineMemo::default()
            });
            cache.len() - 1
        });
        let memo = &mut cache[memo_index];
        let item_debug = memo.debug_strings.entry(item_id).or_default();
        if item_debug.item_version != item_version {
            *item_debug = BaselineDebugStrings {
                item_version,
                ..BaselineDebugStrings::default()
            };
        }
        let cached = match field {
            BaselineDebugField::Kind => &mut item_debug.kind,
            BaselineDebugField::TileShapes => &mut item_debug.tile_shapes,
        };
        if let Some(formatted) = cached {
            return Arc::clone(formatted);
        }
        let formatted = format!("{value:?}");
        let formatted: Arc<str> = Arc::from(formatted);
        *cached = Some(Arc::clone(&formatted));
        formatted
    })
}

fn store_validation_baseline(
    adapter: &BoardAdapter,
    witness: ValidationBaselineWitness,
    violations: Option<Arc<BTreeSet<(i32, i32, i32, u64, u64)>>>,
    connectivity: Option<Arc<BTreeMap<String, Vec<Vec<i32>>>>>,
) {
    VALIDATION_BASELINE_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        if let Some(memo) = cache.iter_mut().find(|memo| {
            memo.witness
                .as_ref()
                .is_some_and(|cached| cached.matches(adapter))
        }) {
            if let Some(violations) = violations {
                memo.violations = Some(violations);
            }
            if let Some(connectivity) = connectivity {
                memo.connectivity = Some(connectivity);
            }
            return;
        }
        if cache.len() == 2 {
            cache.remove(0);
        }
        cache.push(ValidationBaselineMemo {
            witness: Some(witness),
            violations,
            connectivity,
            debug_strings: BTreeMap::new(),
        });
    });
}

fn cached_baseline_violations(
    adapter: &BoardAdapter,
) -> Arc<BTreeSet<(i32, i32, i32, u64, u64)>> {
    let hit = VALIDATION_BASELINE_CACHE.with(|cache| {
        cache.borrow().iter().find_map(|memo| {
            memo.witness
                .as_ref()
                .filter(|witness| witness.matches(adapter))
                .and_then(|_| memo.violations.clone())
        })
    });
    if let Some(violations) = hit {
        return violations;
    }
    let witness = ValidationBaselineWitness::new(adapter);
    let violations = Arc::new(adapter.violations_for_scan());
    store_validation_baseline(adapter, witness, Some(Arc::clone(&violations)), None);
    violations
}

fn cached_baseline_connectivity(
    adapter: &BoardAdapter,
) -> Arc<BTreeMap<String, Vec<Vec<i32>>>> {
    let hit = VALIDATION_BASELINE_CACHE.with(|cache| {
        cache.borrow().iter().find_map(|memo| {
            memo.witness
                .as_ref()
                .filter(|witness| witness.matches(adapter))
                .and_then(|_| memo.connectivity.clone())
        })
    });
    if let Some(connectivity) = hit {
        return connectivity;
    }
    let witness = ValidationBaselineWitness::new(adapter);
    let connectivity = Arc::new(adapter.connectivity());
    store_validation_baseline(adapter, witness, None, Some(Arc::clone(&connectivity)));
    connectivity
}

/// Prove that a baseline item's content and every input used to derive its
/// default-tree shapes are unchanged. Public Arc-backed board data can be
/// mutated through copy-on-write access, so epoch equality alone is not enough.
fn unchanged_debug_geometry_inputs(
    before: &BasicBoard,
    after: &BasicBoard,
    before_key: ItemKey,
    after_key: ItemKey,
) -> bool {
    let before_item = before.item(before_key);
    let after_item = after.item(after_key);
    // Obstacle tiling also depends on Communication's host CAD/resolution
    // settings, which are not represented by the board epochs or Arc fields.
    if before_item.is_obstacle_area()
        || before_item.is_conduction_area()
        || after_item.is_obstacle_area()
        || after_item.is_conduction_area()
    {
        return false;
    }
    (before_key == after_key || before_item.id() == after_item.id())
        && before.item_repository().version(before_key)
            == after.item_repository().version(after_key)
        && before.item_repository().global_epoch() == after.item_repository().global_epoch()
        && std::sync::Arc::ptr_eq(&before.rules, &after.rules)
        && std::sync::Arc::ptr_eq(&before.library, &after.library)
        && std::sync::Arc::ptr_eq(&before.components, &after.components)
        && std::sync::Arc::ptr_eq(&before.layer_structure, &after.layer_structure)
        && before.bounding_box == after.bounding_box
        && before.default_tree().kind == after.default_tree().kind
        && before.default_tree().compensated_clearance_class_no
            == after.default_tree().compensated_clearance_class_no
}

/// Retain native DRC findings unless the broad-phase tile candidate can be
/// disproved against actual copper shapes with the same raw clearance.
/// Keepouts, outlines, and non-authorized structural via/pad restrictions stay native.
fn retain_clearance_candidate(
    board: &BasicBoard,
    violation: &ClearanceViolation,
    allow_awx_same_net_via_pad_contact: bool,
) -> bool {
    let first = board.item(violation.first_item);
    let second = board.item(violation.second_item);
    if first.is_obstacle_area()
        || second.is_obstacle_area()
        || first.is_board_outline()
        || second.is_board_outline()
        || first.is_component_outline()
        || second.is_component_outline()
    {
        return true;
    }
    if board.rules.get_hole_clearance() > 0
        && (first.is_pin() || first.is_via() || second.is_pin() || second.is_via())
    {
        // The search-tree candidate may include a copper-to-hole delta that is
        // not represented by the pad/via copper outlines below.
        return true;
    }

    let via_pin = if first.is_via() && second.is_pin() {
        Some((violation.first_item, violation.second_item))
    } else if first.is_pin() && second.is_via() {
        Some((violation.second_item, violation.first_item))
    } else {
        None
    };
    if let Some((via_key, pin_key)) = via_pin {
        let via = board.item(via_key);
        let pin = board.item(pin_key);
        if via.shares_net(pin) {
            let attach_allowed = via.as_via().is_some_and(|via| via.attach_allowed);
            if !attach_allowed {
                return true;
            }
            if !pin.drill_allowed(board) {
                let same_single_net = via.net_numbers().len() == 1
                    && pin.net_numbers().len() == 1
                    && via.net_numbers()[0] == pin.net_numbers()[0];
                if !(allow_awx_same_net_via_pad_contact && same_single_net) {
                    return true;
                }
                // This is the explicit AWX H2H-disabled same-net via/pad rule,
                // not a numeric copper-clearance result.
                return false;
            }
        }
    }

    let Some(actual_gap) = crate::copper_clearance::actual_copper_gap(
        board,
        violation.first_item,
        violation.second_item,
        violation.layer,
    ) else {
        return true;
    };
    !actual_gap.is_finite() || actual_gap < violation.expected_clearance
}

pub(crate) fn canonical_point_key(point: &fr_geom::Point) -> String {
    match point {
        fr_geom::Point::Int(point) => format!("{},{},1", point.x, point.y),
        fr_geom::Point::Rational(point) => {
            let (x, y, z) = point.normalized_homogeneous_coordinates();
            format!("{x},{y},{z}")
        }
    }
}

#[cfg(test)]
mod canonical_point_key_tests {
    use super::canonical_point_key;
    use fr_geom::{IntPoint, Point, RationalPoint};

    #[test]
    fn equivalent_homogeneous_points_share_a_key() {
        let integer = Point::Int(IntPoint::new(1, 2));
        let rational = Point::Rational(Box::new(RationalPoint::new(2.into(), 4.into(), 2.into())));
        let scaled = Point::Rational(Box::new(RationalPoint::new(3.into(), 6.into(), 3.into())));
        assert_eq!(
            canonical_point_key(&integer),
            canonical_point_key(&rational)
        );
        assert_eq!(canonical_point_key(&rational), canonical_point_key(&scaled));
    }
}

fn canonical_polyline_key(points: &[fr_geom::Point]) -> String {
    let forward: Vec<_> = points.iter().map(canonical_point_key).collect();
    let reverse: Vec<_> = forward.iter().rev().cloned().collect();
    if forward <= reverse {
        forward.join(";")
    } else {
        reverse.join(";")
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct Pad {
    pub reference: String,
    pub chip: String,
    pub pad: String,
    pub net: String,
    pub point: [f64; 2],
    pub layers: Vec<String>,
    #[serde(skip)]
    pub key: Option<ItemKey>,
}
#[derive(Clone)]
pub struct BoardAdapter {
    pub board: RoutingBoard,
    pub units_per_mm: f64,
    source_protected_copper: BTreeSet<i32>,
    authorized_explicit_drills: BTreeMap<(String, String), u64>,
    /// AWX-only rule for same-net via contact on a multilayer pad.
    awx_same_net_via_pad_contact: bool,
    /// Board-outline rings retained by the native DSN loader, in board units.
    pub board_outline_polygons: Vec<Vec<[i32; 2]>>,
    /// True nested DSN outline rings which enclose fewer than two pad centers.
    pub board_cutouts: Vec<Vec<[i32; 2]>>,
    /// Nested outline rings containing two or more pad centers; these are milled
    /// edge contours, not holes through the board.
    pub board_edge_contours: Vec<Vec<[i32; 2]>>,
}
impl BoardAdapter {
    /// Apply the AWX source policy only when the job disables H2H and the native
    /// board has no copper-to-hole clearance rule. Unknown source drill sizes
    /// remain unverified; this does not establish hole safety.
    pub(crate) fn configure_awx_hole_rules(&mut self, job_hole_to_hole_clearance_mm: f64) {
        self.awx_same_net_via_pad_contact =
            job_hole_to_hole_clearance_mm == 0.0 && self.board.rules.get_hole_clearance() == 0;
    }

    pub fn load(bytes: &[u8]) -> Result<Self, String> {
        Self::load_inner(bytes, None, true)
    }
    pub fn load_raw(bytes: &[u8]) -> Result<Self, String> {
        Self::load_inner(bytes, None, false)
    }
    pub fn load_session(bytes: &[u8], session: &[u8]) -> Result<Self, String> {
        Self::load_session_with_via_candidates(bytes, session, &[])
    }
    /// Replay serialized copper against the source DSN plus explicitly authorized
    /// physical via rules. Candidate drills must come from configuration metadata.
    pub fn load_session_with_via_candidates(
        bytes: &[u8],
        session: &[u8],
        candidates: &[(String, f64, f64)],
    ) -> Result<Self, String> {
        let mut loaded = Self::load(bytes)?;
        let known_explicit_drills: BTreeMap<_, _> = loaded
            .board
            .library
            .padstacks
            .iter()
            .map(|stack| {
                (
                    stack.name.clone(),
                    stack.get_explicit_drill_radius().map(f64::to_bits),
                )
            })
            .collect();
        let mut authorized_vias = BTreeMap::new();
        let mut authorized_explicit_drills = BTreeMap::new();
        for (net, diameter, drill) in candidates {
            let info = crate::awx::native_execution::ensure_via_candidate(
                &mut loaded,
                net,
                *diameter,
                *drill,
            )?;
            let padstack_no = loaded.board.rules.via_infos[info].get_padstack();
            let padstack = loaded
                .board
                .library
                .padstacks
                .get(padstack_no)
                .ok_or("authorized SES via padstack missing")?;
            let explicit_drill = padstack
                .get_explicit_drill_radius()
                .ok_or("authorized SES via has no explicit drill metadata")?
                .to_bits();
            if known_explicit_drills
                .iter()
                .find(|(name, _)| name.as_str().eq_ignore_ascii_case(&padstack.name))
                .and_then(|(_, radius)| *radius)
                .is_some_and(|radius| radius != explicit_drill)
            {
                return Err(format!(
                    "authorized via candidate changes known drill metadata for {}",
                    padstack.name
                ));
            }
            authorized_explicit_drills.insert((net.clone(), padstack.name.clone()), explicit_drill);
            authorized_vias.insert((loaded.net_number(net)?, padstack_no), info);
        }
        loaded.authorized_explicit_drills = authorized_explicit_drills;
        let mut source_protected = source_protected_copper_counts(&loaded.board)?;

        replay_full_session(&mut loaded.board, session, &authorized_vias)?;
        loaded.source_protected_copper.clear();
        for key in loaded.board.item_repository().iter() {
            let item = loaded.board.item(key);
            if !(item.is_trace() || item.is_via()) {
                continue;
            }
            let signature = source_copper_signature(&loaded.board, item)?;
            if let Some(count) = source_protected.get_mut(&signature) {
                if *count > 0 {
                    loaded.source_protected_copper.insert(item.id().0);
                    *count -= 1;
                }
            }
        }
        if source_protected.values().any(|&count| count != 0) {
            return Err("SES replay removed source-fixed copper".into());
        }
        let nested = std::mem::take(&mut loaded.board_cutouts)
            .into_iter()
            .chain(std::mem::take(&mut loaded.board_edge_contours))
            .collect();
        let (board_cutouts, board_edge_contours) = classify_nested_contours(&loaded.board, nested);
        loaded.board_cutouts = board_cutouts;
        loaded.board_edge_contours = board_edge_contours;
        Ok(loaded)
    }
    fn load_inner(bytes: &[u8], _session: Option<&[u8]>, normalized: bool) -> Result<Self, String> {
        let expressions = fr_dsn::sexpr::parse(bytes).map_err(|e| e.to_string())?;
        fn check(expr: &fr_dsn::sexpr::Sexpr) -> Result<(), String> {
            if let Some(list) = expr.as_list() {
                if list.is("space_in_quoted_tokens")
                    && list
                        .args()
                        .first()
                        .and_then(|a| a.as_atom())
                        .is_some_and(|a| a.text.eq_ignore_ascii_case("on"))
                {
                    return Err(
                        "space_in_quoted_tokens=ON unsupported; verified normalization required"
                            .into(),
                    );
                }
                if list.is("net")
                    && list.args().iter().filter(|a| a.as_atom().is_some()).count() > 1
                {
                    return Err(
                        "ambiguous multi-atom net name; verified normalization required".into(),
                    );
                }
                for e in &list.items {
                    check(e)?;
                }
            }
            Ok(())
        }
        for expression in &expressions {
            check(expression)?;
        }
        let mut settings = RouterSettings::default();
        let mut board = fr_io::post_load::load_from_specctra_dsn(bytes, &mut settings)
            .map_err(|e| format!("DSN load: {e:?}"))?;
        if normalized {
            let dsn = fr_dsn::Dsn::parse(bytes).map_err(|e| format!("DSN parse: {e:?}"))?;
            fr_io::network::extend_class_pair_clearances(&mut board, &dsn);
            fr_io::post_load::prepare_for_routing(&mut board, &mut settings, None);
            board.keep_wiring_inside_outline();
            fr_engine::pipeline::deferred_post_load_processing(&mut board);
            board.fallback_vias_own_class = true;
            board.set_overlap_contacts(true);
            board.mark_stitching_vias();
            board.bridge_trace_ends_to_drill_centers();
        }
        let units_per_mm = board.communication.coordinate_transform.scale_factor() * 1000.0
            / board.communication.unit.micrometers();
        if !units_per_mm.is_finite() || units_per_mm <= 0.0 {
            return Err("invalid board coordinate scale".into());
        }
        let (board_outline_polygons, board_cutouts, board_edge_contours) =
            board_contour_metadata(bytes, &board)?;
        let source_protected_copper = source_protected_copper_counts(&board)?;
        let mut protected_ids = BTreeSet::new();
        let mut remaining = source_protected_copper;
        for key in board.item_repository().iter() {
            let item = board.item(key);
            if !(item.is_trace() || item.is_via()) {
                continue;
            }
            let signature = source_copper_signature(&board, item)?;
            if let Some(count) = remaining.get_mut(&signature) {
                if *count > 0 {
                    protected_ids.insert(item.id().0);
                    *count -= 1;
                }
            }
        }
        Ok(Self {
            board,
            units_per_mm,
            source_protected_copper: protected_ids,
            authorized_explicit_drills: BTreeMap::new(),
            awx_same_net_via_pad_contact: false,
            board_outline_polygons,
            board_cutouts,
            board_edge_contours,
        })
    }
    pub fn pads(&self) -> Vec<Pad> {
        self.board
            .get_pins()
            .into_iter()
            .filter_map(|key| {
                let item = self.board.item(key);
                let pin = item.as_pin()?;
                let component = self.board.components.get(item.component_no());
                let package = self.board.library.packages.get(component.get_package());
                let pad = package.get_pin(pin.pin_index)?;
                let point = item.center(&self.board).to_float();
                let layers = (item.first_layer(&self.board)..=item.last_layer(&self.board))
                    .filter(|&l| self.board.layer_structure.layers[l as usize].is_signal)
                    .map(|l| self.board.layer_structure.layers[l as usize].name.clone())
                    .collect();
                let net = item
                    .net_numbers()
                    .first()
                    .and_then(|&n| self.board.rules.nets.get(n))
                    .map(|n| n.name.clone())
                    .unwrap_or_default();
                Some(Pad {
                    reference: format!("{}.{}", component.name, pad.name),
                    chip: component.name.clone(),
                    pad: pad.name.clone(),
                    net,
                    point: [point.x / self.units_per_mm, point.y / self.units_per_mm],
                    layers,
                    key: Some(key),
                })
            })
            .collect()
    }
    /// True only for source copper that was fixed when the DSN was loaded.
    /// Generated copper remains transaction-owned even if later fixed.
    pub fn is_source_protected_copper(&self, key: ItemKey) -> bool {
        let item = self.board.item(key);
        (item.is_trace() || item.is_via()) && self.source_protected_copper.contains(&item.id().0)
    }
    fn authorized_drill_upgrade(
        &self,
        source: &BoardAdapter,
        item: &fr_engine::board::Item,
    ) -> Option<u64> {
        let via = item.as_via()?;
        let padstack = source.board.library.padstacks.get(via.padstack_no())?;
        if padstack.get_explicit_drill_radius().is_some() || item.net_numbers().len() != 1 {
            return None;
        }
        let net = source.board.rules.nets.get(item.net_numbers()[0])?;
        self.authorized_explicit_drills
            .get(&(net.name.clone(), padstack.name.clone()))
            .copied()
    }
    pub fn net_number(&self, name: &str) -> Result<NetNo, String> {
        let nets: Vec<_> = self
            .board
            .rules
            .nets
            .iter()
            .filter(|n| n.name == name)
            .collect();
        if nets.len() != 1 {
            return Err(format!(
                "net {name}: expected exactly one DSN net, got {}",
                nets.len()
            ));
        }
        Ok(nets[0].net_number)
    }
    pub fn derived_rule(&self, pad: &Pad) -> Result<NetRule, String> {
        let n = self.net_number(&pad.net)?;
        let class = self.board.rules.net_class_of(n);
        let l = self
            .board
            .layer_structure
            .get_no(pad.layers.first().ok_or("pad has no routable layer")?);
        let cl = class.get_trace_clearance_class();
        let clearance = self
            .board
            .rules
            .clearance_matrix
            .get_value(cl, cl, l, false) as f64
            / self.units_per_mm;
        Ok(NetRule {
            name: pad.net.clone(),
            width_mm: 2.0 * class.get_trace_half_width(l) as f64 / self.units_per_mm,
            clearance_mm: clearance,
            allowed_layers: pad.layers.clone(),
            via_budget: None,
            max_length_mm: None,
            protected: false,
        })
    }
    pub fn point(&self, p: [f64; 2]) -> Result<Point, String> {
        let c = p.map(|x| (x * self.units_per_mm).round());
        if c.iter().any(|x| !x.is_finite() || x.abs() > 100_000_000.0) {
            return Err("coordinate outside safe engine range".into());
        }
        Ok(Point::Int(IntPoint::new(c[0] as i32, c[1] as i32)))
    }
    pub fn quantized(&self, p: [f64; 2]) -> Result<[f64; 2], String> {
        let q = self.point(p)?.to_float();
        Ok([q.x / self.units_per_mm, q.y / self.units_per_mm])
    }
    pub fn ses(&self) -> Vec<u8> {
        fr_io::ses_writer::ses_bytes(&self.board, "fr_agent.dsn")
    }
    pub fn check_candidate(
        &self,
        candidate: &PlanCandidate,
        spec: &RouteSpec,
    ) -> Result<(), String> {
        let mut trial = self.clone();
        trial.insert_candidate(candidate, spec).map(|_| ())
    }
    pub fn insert_candidate(
        &mut self,
        candidate: &PlanCandidate,
        spec: &RouteSpec,
    ) -> Result<Vec<ItemKey>, String> {
        if candidate.id.starts_with("escape:") {
            for leg in &candidate.legs {
                let start = *leg.points.first().ok_or("empty escape")?;
                let end = *leg.points.last().ok_or("empty escape")?;
                let pads = self.pads();
                if !pads.iter().any(|p| {
                    p.net == leg.net
                        && p.layers.contains(&leg.layer)
                        && distance(p.point, start) < 1.0 / self.units_per_mm
                }) {
                    return Err("escape must start at its pad center".into());
                }
                let length: f64 = leg.points.windows(2).map(|s| distance(s[0], s[1])).sum();
                if length > 0.601 || distance(start, end) > 0.601 {
                    return Err("escape exceeds local 0.6 mm region".into());
                }
            }
        }
        let mut inserted = Vec::new();
        for via in &candidate.vias {
            let rule = spec
                .nets
                .iter()
                .find(|r| r.name == via.net)
                .ok_or("via net has no rule")?;
            if rule.protected || rule.via_budget == Some(0) {
                return Err("via violates protected net/via budget".into());
            }
            let n = self.net_number(&via.net)?;
            let from = self.board.layer_structure.get_no(&via.from_layer);
            let to = self.board.layer_structure.get_no(&via.to_layer);
            let pad = self
                .board
                .library
                .padstacks
                .get_by_name(&via.padstack)
                .ok_or("unknown via padstack")?
                .clone();
            if from != pad.from_layer()
                || to != pad.to_layer()
                || from >= to
                || (from..=to).any(|l| {
                    !rule
                        .allowed_layers
                        .contains(&self.board.layer_structure.layers[l as usize].name)
                })
            {
                return Err("via padstack span violates allowed layers".into());
            }
            let bounds = pad
                .get_shape(from)
                .ok_or("via padstack has no copper")?
                .bounding_box();
            let diameter = bounds.width().max(bounds.height()) as f64 / self.units_per_mm;
            if !via.diameter_mm.is_finite()
                || (diameter - via.diameter_mm).abs() > 1.0 / self.units_per_mm
            {
                return Err("via diameter does not match native padstack".into());
            }
            if let Some(group) = spec.groups.iter().find(|g| g.id == candidate.group_id) {
                if !group.channels.is_empty() {
                    let radius = diameter / 2.0 + rule.clearance_mm;
                    if !group
                        .channels
                        .iter()
                        .filter_map(|id| spec.channels.iter().find(|c| &c.id == id))
                        .any(|c| segment_in_channel(via.point, via.point, radius, &c.region))
                    {
                        return Err("via outside channel transition zone".into());
                    }
                }
            }
            let via_rule = self
                .board
                .rules
                .net_class_of(n)
                .get_via_rule()
                .or_else(|| self.board.rules.via_rules.get_first())
                .ok_or("net class permits no vias")?;
            let info_id = self.board.rules.via_rules[via_rule]
                .vias()
                .iter()
                .copied()
                .find(|&id| self.board.rules.via_infos[id].get_padstack() == pad.id)
                .ok_or("via padstack is not authorized by this net class via rule")?;
            let mut info = self.board.rules.via_infos[info_id].clone();
            let desired = checked_size(rule.clearance_mm * self.units_per_mm)?;
            let base = info.get_clearance_class_index();
            let class_name = format!("fr_agent_via_{}_{}", n, desired);
            let matrix = &mut self.board.rules_mut().clearance_matrix;
            matrix.append_class(&class_name);
            let cl = matrix.get_no(&class_name);
            for l in from..=to {
                for other in 0..matrix.get_class_count() {
                    let old = matrix
                        .get_value(base, other, l, false)
                        .max(matrix.get_value(other, base, l, false));
                    matrix.set_value(cl, other, l, old.max(desired));
                    matrix.set_value(other, cl, l, old.max(desired));
                }
            }
            info.set_clearance_class_index(cl);
            let location = self.point(via.point)?;
            let widths = vec![
                checked_size(rule.width_mm * self.units_per_mm / 2.0)?;
                self.board.layer_count() as usize
            ];
            let previous_via_ids: BTreeSet<_> = self
                .board
                .item_repository()
                .iter()
                .filter(|&k| self.board.item(k).is_via())
                .map(|k| self.board.item(k).id())
                .collect();
            if !self.board.forced_via(
                &info,
                &location,
                &[n],
                info.get_clearance_class_index(),
                &widths,
                0,
                0,
                0,
                0,
                0,
            ) {
                return Err("engine rejected checked via".into());
            }
            let new_vias: Vec<_> = self
                .board
                .item_repository()
                .iter()
                .filter(|&k| {
                    self.board.item(k).is_via()
                        && !previous_via_ids.contains(&self.board.item(k).id())
                })
                .collect();
            for key in new_vias {
                self.board
                    .item_mut(key)
                    .set_fixed_state(FixedState::UserFixed);
            }
            let count = self
                .board
                .item_repository()
                .iter()
                .filter(|&k| self.board.item(k).is_via() && self.board.item(k).contains_net(n))
                .count();
            if count > rule.via_budget.unwrap_or(u32::MAX) as usize {
                return Err("via budget exceeded".into());
            }
        }

        for leg in &candidate.legs {
            inserted.push(self.insert_leg(
                leg,
                spec,
                &candidate.group_id,
                !candidate.id.starts_with("escape:"),
            )?);
        }
        Ok(inserted)
    }
    fn insert_leg(
        &mut self,
        leg: &RouteLeg,
        spec: &RouteSpec,
        group: &str,
        enforce_channel: bool,
    ) -> Result<ItemKey, String> {
        if spec.protected_names().contains(&leg.net) {
            return Err(format!("protected net {}", leg.net));
        }
        let n = self.net_number(&leg.net)?;
        let layer = self.board.layer_structure.get_no(&leg.layer);
        if layer < 0 || !self.board.layer_structure.layers[layer as usize].is_signal {
            return Err(format!("{}: invalid signal layer {}", leg.net, leg.layer));
        }
        let rule = spec.nets.iter().find(|r| r.name == leg.net);
        if let Some(r) = rule {
            if r.protected {
                return Err(format!("protected net {}", leg.net));
            }
            if !r.allowed_layers.contains(&leg.layer)
                || leg.width_mm + 1e-12 < r.width_mm
                || leg.clearance_mm + 1e-12 < r.clearance_mm
            {
                return Err(format!(
                    "{}: layer/width/clearance hard rule violated",
                    leg.net
                ));
            }
        }
        if leg.points.len() < 2
            || leg.points.iter().flatten().any(|x| !x.is_finite())
            || !leg.width_mm.is_finite()
            || leg.width_mm <= 0.0
            || !leg.clearance_mm.is_finite()
            || leg.clearance_mm < 0.0
        {
            return Err("invalid polyline geometry".into());
        }
        let points: Vec<Point> = leg
            .points
            .iter()
            .map(|&p| self.point(p))
            .collect::<Result<_, _>>()?;
        if points.windows(2).any(|s| s[0] == s[1]) {
            return Err("quantized segment collapsed".into());
        }
        for segment in points.windows(2) {
            let a = segment[0].to_float();
            let b = segment[1].to_float();
            let dx = (b.x - a.x).abs();
            let dy = (b.y - a.y).abs();
            let legal = match self.board.rules.get_trace_angle_restriction() {
                fr_engine::ids::AngleRestriction::None => true,
                fr_engine::ids::AngleRestriction::NinetyDegree => dx == 0.0 || dy == 0.0,
                fr_engine::ids::AngleRestriction::FortyfiveDegree => {
                    dx == 0.0 || dy == 0.0 || dx == dy
                }
            };
            if !legal {
                return Err("candidate violates native trace angle restriction".into());
            }
        }
        let half = checked_size(leg.width_mm * self.units_per_mm / 2.0)?;
        let clearance = checked_size(leg.clearance_mm * self.units_per_mm)?;
        let envelope = half as f64 / self.units_per_mm + clearance as f64 / self.units_per_mm;
        if enforce_channel {
            if let Some(g) = spec.groups.iter().find(|g| g.id == group) {
                if !g.channels.is_empty() {
                    let channels: Vec<_> = g
                        .channels
                        .iter()
                        .filter_map(|id| spec.channels.iter().find(|c| &c.id == id))
                        .collect();
                    for s in points.windows(2) {
                        let a = s[0].to_float();
                        let b = s[1].to_float();
                        if !channels.iter().any(|c| {
                            segment_in_channel(
                                [a.x / self.units_per_mm, a.y / self.units_per_mm],
                                [b.x / self.units_per_mm, b.y / self.units_per_mm],
                                envelope,
                                &c.region,
                            )
                        }) {
                            return Err(format!(
                                "{}: segment outside hard channel envelope",
                                leg.net
                            ));
                        }
                    }
                }
            }
        }
        let base_class = self.board.rules.net_class_of(n).get_trace_clearance_class();
        let class_name = format!("fr_agent_{}_{}_{}", n, half, clearance);
        let matrix = &mut self.board.rules_mut().clearance_matrix;
        matrix.append_class(&class_name);
        let cl = matrix.get_no(&class_name);
        let count = matrix.get_class_count();
        for other in 0..count {
            let old = matrix
                .get_value(base_class, other, layer, false)
                .max(matrix.get_value(other, base_class, layer, false));
            matrix.set_value(cl, other, layer, old.max(clearance));
            matrix.set_value(other, cl, layer, old.max(clearance));
        }
        let previous_tolerance = self.board.rules.clearance_tolerance_um;
        self.board.rules_mut().clearance_tolerance_um = 0.0;
        let poly = Polyline::from_points(&points);
        if !self
            .board
            .check_polyline_trace(&poly, layer, half, &[n], cl)
        {
            return Err(format!(
                "{}: engine clearance/keepout/outline check rejected candidate",
                leg.net
            ));
        }
        let key = self
            .board
            .insert_trace_without_cleaning(poly, layer, half, &[n], cl, FixedState::UserFixed)
            .ok_or("engine rejected trace insertion")?;
        if clearance_violations(&self.board, key)
            .iter()
            .any(|v| retain_clearance_candidate(&self.board, v, self.awx_same_net_via_pad_contact))
        {
            return Err(format!("{}: inserted candidate violates DRC", leg.net));
        }
        self.board.rules_mut().clearance_tolerance_um = previous_tolerance;
        Ok(key)
    }
    pub fn violations(&self) -> BTreeSet<(i32, i32, i32, u64, u64)> {
        self.violations_for_scan()
    }
    fn violations_for_scan(&self) -> BTreeSet<(i32, i32, i32, u64, u64)> {
        let mut result = BTreeSet::new();
        for key in self.board.item_repository().iter() {
            for candidate in clearance_violations(&self.board, key) {
                if retain_clearance_candidate(
                    &self.board,
                    &candidate,
                    self.awx_same_net_via_pad_contact,
                ) {
                    let a = self.board.item(candidate.first_item).id().0;
                    let b = self.board.item(candidate.second_item).id().0;
                    result.insert((
                        a.min(b),
                        a.max(b),
                        candidate.layer,
                        candidate.expected_clearance.to_bits(),
                        candidate.actual_clearance.to_bits(),
                    ));
                }
            }
        }
        result
    }
    /// Exact-copper DRC additions relative to `before`, excluding only a
    /// redundant full-width center bridge into an unchanged source pad when
    /// the same owner/other pin pair already violated clearance at baseline.
    pub fn new_violations_since(
        &self,
        before: &BoardAdapter,
    ) -> BTreeSet<(i32, i32, i32, u64, u64)> {
        let baseline = cached_baseline_violations(before);
        let after = self.violations_for_scan();
        let mut added = after.difference(&baseline).copied().collect::<BTreeSet<_>>();
        if self.board.rules.get_hole_clearance() > 0
            || before.board.rules.get_hole_clearance() > 0
            || added.is_empty()
        {
            return added;
        }

        let after_items = self
            .board
            .get_items()
            .into_iter()
            .map(|key| (self.board.item(key).id().0, key))
            .collect::<BTreeMap<_, _>>();
        let before_items = before
            .board
            .get_items()
            .into_iter()
            .map(|key| (before.board.item(key).id().0, key))
            .collect::<BTreeMap<_, _>>();
        let additions = added.iter().copied().collect::<Vec<_>>();
        for entry @ (_, _, layer, expected_bits, _) in additions {
            let (Some(&first), Some(&second)) =
                (after_items.get(&entry.0), after_items.get(&entry.1))
            else {
                continue;
            };
            let first_item = self.board.item(first);
            let second_item = self.board.item(second);
            let (trace, other_pin) = if first_item.is_trace() && second_item.is_pin() {
                (first, second)
            } else if second_item.is_trace() && first_item.is_pin() {
                (second, first)
            } else {
                continue;
            };
            if self.center_bridge_pair_is_baseline_proven(
                before,
                trace,
                other_pin,
                layer,
                f64::from_bits(expected_bits),
                &baseline,
                &before_items,
            ) {
                added.remove(&entry);
            }
        }
        added
    }

    fn center_bridge_pair_is_baseline_proven(
        &self,
        before: &BoardAdapter,
        trace_key: ItemKey,
        other_pin_key: ItemKey,
        layer: i32,
        expected_clearance: f64,
        baseline_violations: &BTreeSet<(i32, i32, i32, u64, u64)>,
        before_items: &BTreeMap<i32, ItemKey>,
    ) -> bool {
        if before.board.rules.get_hole_clearance() > 0
            || self.board.rules.get_hole_clearance() > 0
            || !other_pin_key_is_unchanged(self, before, other_pin_key, before_items)
        {
            return false;
        }
        let trace = self.board.item(trace_key);
        let other_pin = self.board.item(other_pin_key);
        if !trace.is_trace()
            || !trace.is_on_board()
            || trace.net_numbers().len() != 1
            || !other_pin.is_pin()
            || other_pin.net_numbers().len() != 1
            || trace.net_numbers()[0] == other_pin.net_numbers()[0]
        {
            return false;
        }
        let endpoints = [trace.first_corner(), trace.last_corner()];
        let mut owners = BTreeSet::new();
        for endpoint in endpoints {
            for owner_key in self.board.pick_items(&endpoint, layer, None).iter() {
                let owner = self.board.item(owner_key);
                if owner.is_pin()
                    && owner.is_on_layer(&self.board, layer)
                    && owner.center(&self.board) == endpoint
                    && owner.net_numbers() == trace.net_numbers()
                    && owner_key != other_pin_key
                {
                    owners.insert(owner_key);
                }
            }
        }
        for owner_key in owners {
            if !crate::copper_clearance::contained_pad_center_bridge(
                &self.board,
                trace_key,
                owner_key,
                layer,
            ) {
                continue;
            }
            if !other_pin_key_is_unchanged(self, before, owner_key, before_items) {
                continue;
            }
            let Some(before_owner) = matching_baseline_pin(self, before, owner_key, before_items)
            else {
                continue;
            };
            let Some(before_other) = matching_baseline_pin(self, before, other_pin_key, before_items)
            else {
                continue;
            };
            if before_owner == before_other {
                continue;
            }
            let layer_name = self
                .board
                .layer_structure
                .layers
                .get(layer as usize)
                .map(|layer| layer.name.as_str());
            let Some(before_layer) = layer_name.map(|name| before.board.layer_structure.get_no(name))
            else {
                continue;
            };
            if before_layer < 0 {
                continue;
            }
            let owner_id = before.board.item(before_owner).id().0;
            let other_id = before.board.item(before_other).id().0;
            let pair = (owner_id.min(other_id), owner_id.max(other_id));
            let has_baseline_pin_pair = baseline_violations.iter().any(
                |(a, b, old_layer, expected_bits, _)| {
                    (*a, *b) == pair
                        && *old_layer == before_layer
                        && f64::from_bits(*expected_bits) >= expected_clearance
                },
            );
            if has_baseline_pin_pair {
                return true;
            }
        }
        false
    }

    /// Clearance violations with item identity replaced by canonical geometry
    /// signatures, so SES replay can compare inherited violations after re-ID.
    pub fn session_violations(&self) -> BTreeSet<String> {
        self.session_violations_for(self, false)
    }
    /// Stable clearance violations newly introduced since a source snapshot,
    /// applying only this replay's explicitly authorized drill enrichment.
    pub fn new_session_violations_since(&self, before: &BoardAdapter) -> BTreeSet<String> {
        let mut added = self
            .session_violations()
            .difference(&self.session_violations_for(before, true))
            .cloned()
            .collect::<BTreeSet<_>>();
        if self.board.rules.get_hole_clearance() > 0
            || before.board.rules.get_hole_clearance() > 0
            || added.is_empty()
        {
            return added;
        }
        let baseline = cached_baseline_violations(before);
        let before_items = before
            .board
            .get_items()
            .into_iter()
            .map(|key| (before.board.item(key).id().0, key))
            .collect::<BTreeMap<_, _>>();
        for trace_key in self.board.get_traces() {
            for violation in clearance_violations(&self.board, trace_key) {
                if !retain_clearance_candidate(
                    &self.board,
                    &violation,
                    self.awx_same_net_via_pad_contact,
                ) {
                    continue;
                }
                let first = self.board.item(violation.first_item);
                let second = self.board.item(violation.second_item);
                let (trace, other_pin) =
                    if first.is_trace() && second.is_pin() {
                        (violation.first_item, violation.second_item)
                    } else if second.is_trace() && first.is_pin() {
                        (violation.second_item, violation.first_item)
                    } else {
                        continue;
                    };
                let signature = self.session_violation_signature(
                    self,
                    &violation,
                    false,
                );
                if added.contains(&signature)
                    && self.center_bridge_pair_is_baseline_proven(
                        before,
                        trace,
                        other_pin,
                        violation.layer,
                        violation.expected_clearance,
                        &baseline,
                        &before_items,
                    )
                {
                    added.remove(&signature);
                }
            }
        }
        added
    }
    fn session_violations_for(
        &self,
        source: &BoardAdapter,
        allow_trusted_drill_upgrade: bool,
    ) -> BTreeSet<String> {
        source
            .board
            .item_repository()
            .iter()
            .flat_map(|key| clearance_violations(&source.board, key))
            .filter(|violation| {
                retain_clearance_candidate(
                    &source.board,
                    violation,
                    self.awx_same_net_via_pad_contact && source.awx_same_net_via_pad_contact,
                )
            })
            .map(|violation| {
                self.session_violation_signature(source, &violation, allow_trusted_drill_upgrade)
            })
            .collect()
    }
    fn session_violation_signature(
        &self,
        source: &BoardAdapter,
        violation: &ClearanceViolation,
        allow_trusted_drill_upgrade: bool,
    ) -> String {
        let signature = |key| {
            let item = source.board.item(key);
            let authorized = allow_trusted_drill_upgrade
                .then(|| self.authorized_drill_upgrade(source, item))
                .flatten();
            session_item_signature(&source.board, item, authorized)
        };
        let mut items = [signature(violation.first_item), signature(violation.second_item)];
        items.sort();
        let layer = source.board.layer_structure.layers[violation.layer as usize]
            .name
            .clone();
        format!(
            "{}|{}|{}|{:016x}|{:016x}",
            items[0],
            items[1],
            layer,
            violation.expected_clearance.to_bits(),
            violation.actual_clearance.to_bits()
        )
    }
    pub fn connectivity(&self) -> BTreeMap<String, Vec<Vec<i32>>> {
        self.board
            .rules
            .nets
            .iter()
            .map(|n| {
                let connected_sets = self.board.get_connected_sets_cached(n.net_number);
                let mut groups: Vec<Vec<i32>> = connected_sets
                    .into_iter()
                    .map(|s| {
                        let mut ids: Vec<_> = s
                            .iter()
                            .filter(|&k| self.board.item(k).is_pin())
                            .map(|k| self.board.item(k).id().0)
                            .collect();
                        ids.sort();
                        ids
                    })
                    .filter(|s: &Vec<_>| !s.is_empty())
                    .collect();
                groups.sort();
                (n.name.clone(), groups)
            })
            .collect()
    }
    pub fn assert_preserved(&self, before: &Self) -> Result<(), String> {
        self.assert_invariants(before, &BTreeSet::new(), true)
    }
    /// Validate a fully replayed SES while allowing its routing copper item IDs
    /// to differ from the source board. Required original copper is compared as
    /// an exact multiset so duplicate segments cannot disappear unnoticed.
    pub fn assert_session_invariants(
        &self,
        before: &Self,
        protected: &BTreeSet<String>,
        strict: bool,
    ) -> Result<(), String> {
        fn copper_signature(
            adapter: &BoardAdapter,
            item: &fr_engine::board::Item,
            authorized_drill_upgrade: Option<u64>,
        ) -> Result<String, String> {
            let board = &adapter.board;
            let nets: Vec<_> = item
                .net_numbers()
                .iter()
                .map(|&n| {
                    board
                        .rules
                        .nets
                        .get(n)
                        .map(|net| net.name.clone())
                        .ok_or_else(|| format!("routing copper references unknown net {n:?}"))
                })
                .collect::<Result<_, _>>()?;
            let layers: Vec<_> = (item.first_layer(board)..=item.last_layer(board))
                .map(|l| board.layer_structure.layers[l as usize].name.as_str())
                .collect();
            let geometry = if item.is_trace() {
                let trace = item.trace();
                format!(
                    "trace:{}:{:?}",
                    canonical_polyline_key(&trace.polyline().corners()),
                    trace.half_width()
                )
            } else {
                let _ = item.as_via().expect("routing copper is trace or via");
                format!(
                    "via:{}:{}",
                    via_physical_signature(board, item, authorized_drill_upgrade)?,
                    canonical_point_key(&item.center(board))
                )
            };
            Ok(format!(
                "{nets:?}|{layers:?}|{geometry}|{:?}|{:?}",
                item.fixed_state(),
                item.clearance_class()
            ))
        }
        fn is_protected(
            adapter: &BoardAdapter,
            item: &fr_engine::board::Item,
            protected: &BTreeSet<String>,
        ) -> Result<bool, String> {
            for &n in item.net_numbers() {
                let net = adapter
                    .board
                    .rules
                    .nets
                    .get(n)
                    .ok_or_else(|| format!("routing copper references unknown net {n:?}"))?;
                if protected.contains(&net.name) {
                    return Ok(true);
                }
            }
            Ok(false)
        }
        fn add_count(counts: &mut BTreeMap<String, usize>, signature: String) {
            *counts.entry(signature).or_default() += 1;
        }

        let mut required = BTreeMap::new();
        let mut protected_original = BTreeMap::new();
        for key in before.board.item_repository().iter() {
            let item = before.board.item(key);
            if item.is_trace() || item.is_via() {
                let protected_item = is_protected(before, item, protected)?;
                if strict || item.fixed_state() != FixedState::Unfixed || protected_item {
                    add_count(
                        &mut required,
                        copper_signature(
                            before,
                            item,
                            self.authorized_drill_upgrade(before, item),
                        )?,
                    );
                }
                if protected_item {
                    add_count(
                        &mut protected_original,
                        copper_signature(
                            before,
                            item,
                            self.authorized_drill_upgrade(before, item),
                        )?,
                    );
                }
                continue;
            }

            let now_key = self
                .board
                .item_repository()
                .iter()
                .find(|&candidate| self.board.item(candidate).id() == item.id())
                .ok_or("original non-routing item removed")?;
            let now = self.board.item(now_key);
            if item.board_item_type() != now.board_item_type()
                || item.component_no() != now.component_no()
                || item.first_layer(&before.board) != now.first_layer(&self.board)
                || item.last_layer(&before.board) != now.last_layer(&self.board)
                || format!("{:?}", before.board.item_tile_shapes(item))
                    != format!("{:?}", self.board.item_tile_shapes(now))
                || format!("{:?}", item.kind) != format!("{:?}", now.kind)
                || item.net_numbers() != now.net_numbers()
                || item.fixed_state() != now.fixed_state()
                || item.clearance_class() != now.clearance_class()
            {
                return Err("original non-routing entity properties changed".into());
            }
        }

        let mut actual_required = BTreeMap::new();
        let mut protected_actual = BTreeMap::new();
        for key in self.board.item_repository().iter() {
            let item = self.board.item(key);
            if item.is_trace() || item.is_via() {
                let signature = copper_signature(self, item, None)?;
                if required.contains_key(&signature) {
                    add_count(&mut actual_required, signature.clone());
                }
                if is_protected(self, item, protected)? {
                    add_count(&mut protected_actual, signature);
                }
            }
        }
        if required != actual_required {
            return Err("required original copper geometry/classification changed".into());
        }
        if protected_original != protected_actual {
            return Err("protected copper changed or acquired new geometry".into());
        }
        if !self.new_session_violations_since(before).is_empty() {
            return Err("new full-board clearance violation".into());
        }
        let previous = before.connectivity();
        let current = self.connectivity();
        for (net, groups) in previous {
            for connected in groups {
                if !current.get(&net).is_some_and(|sets| {
                    sets.iter()
                        .any(|set| connected.iter().all(|id| set.contains(id)))
                }) {
                    return Err(format!("existing {net} connectivity split"));
                }
            }
        }
        Ok(())
    }
    pub fn assert_invariants(
        &self,
        before: &Self,
        protected: &BTreeSet<String>,
        strict: bool,
    ) -> Result<(), String> {
        self.assert_invariants_with_moved_components(before, protected, strict, &BTreeSet::new())
    }
    pub fn assert_invariants_with_moved_components(
        &self,
        before: &Self,
        protected: &BTreeSet<String>,
        strict: bool,
        moved_components: &BTreeSet<String>,
    ) -> Result<(), String> {
        let after: BTreeMap<_, _> = self
            .board
            .item_repository()
            .iter()
            .map(|k| {
                let item = self.board.item(k);
                (item.id().0, k)
            })
            .collect();
        for k in before.board.item_repository().iter() {
            let item = before.board.item(k);
            let component_no = item.component_no();
            let moved_component = if component_no <= 0 {
                false
            } else if component_no > before.board.components.count() {
                return Err(format!(
                    "original item references missing component {component_no}"
                ));
            } else {
                let component = before.board.components.get(component_no);
                moved_components.contains(&component.name)
            };
            if !strict
                && (item.is_trace() || item.is_via())
                && item.fixed_state() == FixedState::Unfixed
                && !item.net_numbers().iter().any(|&n| {
                    before
                        .board
                        .rules
                        .nets
                        .get(n)
                        .is_some_and(|net| protected.contains(&net.name))
                })
            {
                continue;
            }
            let a = after.get(&item.id().0).ok_or("original item removed")?;
            let now = self.board.item(*a);
            let debug_geometry_unchanged = if moved_component {
                false
            } else {
                let unchanged = unchanged_debug_geometry_inputs(
                    &before.board,
                    &self.board,
                    k,
                    *a,
                );
                unchanged
            };
            if item.board_item_type() != now.board_item_type()
                || item.component_no() != now.component_no()
                || item.first_layer(&before.board) != now.first_layer(&self.board)
                || item.last_layer(&before.board) != now.last_layer(&self.board)
                || (!moved_component
                    && !debug_geometry_unchanged
                    && cached_baseline_debug(
                        before,
                        k,
                        BaselineDebugField::TileShapes,
                        &before.board.item_tile_shapes(item),
                    )
                    .as_ref()
                        != format!("{:?}", self.board.item_tile_shapes(now)).as_str())
            {
                return Err("frozen original item type/shape/layers changed".into());
            }
            if !moved_component
                && !debug_geometry_unchanged
                && !item.is_trace()
                && !item.is_via()
                && cached_baseline_debug(
                    before,
                    k,
                    BaselineDebugField::Kind,
                    &item.kind,
                )
                .as_ref()
                    != format!("{:?}", now.kind).as_str()
            {
                return Err("original non-routing entity properties changed".into());
            }
            if item.net_numbers() != now.net_numbers()
                || item.fixed_state() != now.fixed_state()
                || item.clearance_class() != now.clearance_class()
            {
                return Err("original item's nets/fixed state/clearance modified".into());
            }
            if item.is_trace() {
                if item.trace().polyline().corners() != now.trace().polyline().corners()
                    || item.trace().half_width() != now.trace().half_width()
                    || item.trace().layer() != now.trace().layer()
                {
                    return Err("original trace geometry modified".into());
                }
            } else if item.is_pin() || item.is_via() {
                if moved_component && item.is_pin() {
                    let old = item.center(&before.board).to_float();
                    let new = now.center(&self.board).to_float();
                    let old_component = before
                        .board
                        .components
                        .get(item.component_no())
                        .get_location()
                        .ok_or("moved component had no original placement")?
                        .to_float();
                    let new_component = self
                        .board
                        .components
                        .get(now.component_no())
                        .get_location()
                        .ok_or("moved component has no output placement")?
                        .to_float();
                    if ((new.x - old.x) - (new_component.x - old_component.x)).abs() > 1.0
                        || ((new.y - old.y) - (new_component.y - old_component.y)).abs() > 1.0
                    {
                        return Err("authorized component translation did not move its pad with the component".into());
                    }
                    continue;
                }
                if item.center(&before.board) != now.center(&self.board) {
                    return Err("original pad/via moved".into());
                }
                if item.is_via()
                    && item.as_via().unwrap().padstack_no()
                        != now
                            .as_via()
                            .ok_or("original via type changed")?
                            .padstack_no()
                {
                    return Err("original via padstack changed".into());
                }
            }
        }
        let original_ids: BTreeSet<_> = before
            .board
            .item_repository()
            .iter()
            .map(|k| before.board.item(k).id())
            .collect();
        let mut protected_copper_found = false;
        for k in self.board.item_repository().iter() {
            let item = self.board.item(k);
            if !original_ids.contains(&item.id())
                && (item.is_trace() || item.is_via())
                && item.net_numbers().iter().any(|&n| {
                    self.board
                        .rules
                        .nets
                        .get(n)
                        .is_some_and(|net| protected.contains(&net.name))
                })
            {
                protected_copper_found = true;
                break;
            }
        }
        if protected_copper_found {
            return Err("protected net acquired new copper".into());
        }
        if !self.new_violations_since(before).is_empty() {
            return Err("new full-board clearance violation".into());
        }
        let previous = cached_baseline_connectivity(before);
        let current = self.connectivity();
        for (net, groups) in previous.iter() {
            for connected in groups {
                let mut found = false;
                if let Some(sets) = current.get(net) {
                    for set in sets {
                        let mut contains_all = true;
                        for id in connected {
                            // `connectivity()` sorts every pin-id vector before returning it,
                            // so use logarithmic lookup when checking whether an old group is
                            // still contained in a current group.
                            if set.binary_search(id).is_err() {
                                contains_all = false;
                                break;
                            }
                        }
                        if contains_all {
                            found = true;
                            break;
                        }
                    }
                }
                if !found {
                    return Err(format!("existing {net} connectivity split"));
                }
            }
        }
        Ok(())
    }
}

/// Keep the actual DSN outline identity before the loader's holes are inserted
/// as generic obstacle areas. Boundary grouping, layer resolution, transform,
/// and nesting use the same source helpers/decisions as `fr_io::structure`.
fn board_contour_metadata(
    bytes: &[u8],
    board: &RoutingBoard,
) -> Result<(Vec<Vec<[i32; 2]>>, Vec<Vec<[i32; 2]>>, Vec<Vec<[i32; 2]>>), String> {
    use fr_dsn::model::ShapeKind;
    use fr_geom::PolylineShape;
    use fr_io::shapes::{self, PLayer};

    let dsn = fr_dsn::Dsn::parse(bytes).map_err(|e| format!("DSN outline metadata: {e}"))?;
    let expressions = fr_dsn::sexpr::parse(bytes).map_err(|e| e.to_string())?;
    let root = expressions
        .first()
        .and_then(fr_dsn::sexpr::Sexpr::as_list)
        .ok_or("DSN outline metadata has no pcb root")?;
    let order = fr_io::SourceOrder::from_pcb(root, &dsn);

    // Match Structure.read_structure: the first PCB boundary is the bounding
    // shape; paths and later PCB/SIGNAL boundaries are actual outline rings.
    let mut bounding_shape = None;
    let mut outline_kinds = Vec::new();
    for group in &order.boundary_groups {
        let sequence = group.iter().skip(1).chain(group.first());
        for &index in sequence {
            let Some(shape) = dsn.structure.boundaries.get(index) else {
                continue;
            };
            let Some(resolved) = shapes::resolve(shape, None) else {
                continue;
            };
            if resolved.is_path() {
                outline_kinds.push(resolved.kind);
                continue;
            }
            match resolved.layer {
                Some(PLayer::Pcb) if bounding_shape.is_none() => {
                    bounding_shape = Some(resolved.kind);
                }
                Some(PLayer::Pcb | PLayer::Signal) => outline_kinds.push(resolved.kind),
                _ => {}
            }
        }
    }

    let transform_outline = |kind: &ShapeKind| -> Result<Option<PolylineShape>, String> {
        let normalized = match kind {
            ShapeKind::PolygonPath { width, coords } if *width != 0.0 => ShapeKind::PolygonPath {
                width: 0.0,
                coords: coords.clone(),
            },
            other => other.clone(),
        };
        let shape =
            shapes::transform_to_board(&normalized, &board.communication.coordinate_transform)
                .map_err(|e| format!("DSN outline transform: {e}"))?;
        let Some(shape) = shape else {
            return Ok(None);
        };
        Ok(shape.to_polyline_shape().filter(|p| p.dimension() > 0))
    };
    let mut outline_shapes = Vec::new();
    for kind in &outline_kinds {
        if let Some(shape) = transform_outline(kind)? {
            outline_shapes.push(shape);
        }
    }
    // Structure.create_board falls back to its bounding shape only when no
    // explicit outline shape survived conversion.
    if outline_shapes.is_empty() {
        if let Some(kind) = bounding_shape {
            if let Some(shape) = transform_outline(kind)? {
                outline_shapes.push(shape);
            }
        }
    }

    let rings = |shapes: &[PolylineShape]| -> Vec<Vec<[i32; 2]>> {
        shapes
            .iter()
            .map(|shape| {
                let mut ring: Vec<_> = (0..shape.border_line_count())
                    .map(|i| {
                        let p = shape.corner(i).to_float();
                        [p.x.round() as i32, p.y.round() as i32]
                    })
                    .collect();
                if ring.len() > 1 && ring.first() == ring.last() {
                    ring.pop();
                }
                ring
            })
            .filter(|ring| ring.len() >= 3)
            .collect()
    };
    // The live outline exactly reflects fallback handling, transforms, and
    // outline insertion performed by the loader.
    let board_outline_polygons = board
        .get_outline()
        .and_then(|key| board.item(key).as_board_outline())
        .map(|outline| rings(outline.shapes()))
        .unwrap_or_default();

    // Replicate `Structure.separate_holes`: only shapes wholly contained by a
    // non-hole outline are nested cutouts. Preserve that identity before those
    // shapes become per-layer generic `ObstacleArea`s.
    let bounds: Vec<_> = outline_shapes
        .iter()
        .map(PolylineShape::bounding_box)
        .collect();
    let convex: Vec<_> = outline_shapes
        .iter()
        .map(PolylineShape::split_to_convex)
        .collect();
    let mut is_hole = vec![false; outline_shapes.len()];
    for i in 0..outline_shapes.len() {
        for j in 0..outline_shapes.len() {
            if i == j || is_hole[j] || !bounds[i].is_contained_in(&bounds[j]) {
                continue;
            }
            let Some(container) = convex[j].as_ref() else {
                continue;
            };
            let contained = (0..outline_shapes[i].border_line_count()).all(|corner| {
                let point = outline_shapes[i].corner(corner);
                container.iter().any(|piece| piece.contains(&point))
            });
            if contained {
                is_hole[i] = true;
            }
        }
    }
    let mut nested_contours = Vec::new();
    for (index, shape) in outline_shapes.iter().enumerate() {
        if !is_hole[index] {
            continue;
        }
        let ring = rings(std::slice::from_ref(shape)).pop().unwrap_or_default();
        if ring.is_empty() {
            continue;
        }
        nested_contours.push(ring);
    }
    let (board_cutouts, board_edge_contours) = classify_nested_contours(board, nested_contours);
    Ok((board_outline_polygons, board_cutouts, board_edge_contours))
}

fn point_inside_ring(point: [f64; 2], ring: &[[i32; 2]]) -> bool {
    if ring.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = ring.len() - 1;
    for i in 0..ring.len() {
        let (xi, yi) = (ring[i][0] as f64, ring[i][1] as f64);
        let (xj, yj) = (ring[j][0] as f64, ring[j][1] as f64);
        if (yi > point[1]) != (yj > point[1])
            && point[0] < (xj - xi) * (point[1] - yi) / (yj - yi) + xi
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

fn classify_nested_contours(
    board: &RoutingBoard,
    contours: Vec<Vec<[i32; 2]>>,
) -> (Vec<Vec<[i32; 2]>>, Vec<Vec<[i32; 2]>>) {
    let pad_centers: Vec<_> = board
        .get_pins()
        .into_iter()
        .map(|key| {
            let point = board.item(key).center(board).to_float();
            [point.x, point.y]
        })
        .collect();
    let mut cutouts = Vec::new();
    let mut edge_contours = Vec::new();
    for ring in contours.into_iter().filter(|ring| ring.len() >= 3) {
        let enclosed_pads = pad_centers
            .iter()
            .filter(|&&point| point_inside_ring(point, &ring))
            .take(2)
            .count();
        if enclosed_pads >= 2 {
            edge_contours.push(ring);
        } else {
            cutouts.push(ring);
        }
    }
    (cutouts, edge_contours)
}
/// Conservative certification: an entire segment and its copper/clearance envelope must fit
/// in one convex rectangle or one straight capsule of a channel polyline.
pub fn segment_in_channel(a: [f64; 2], b: [f64; 2], radius: f64, c: &ChannelRegion) -> bool {
    match c {
        ChannelRegion::Rectangle { min, max } => [a, b].iter().all(|p| {
            p[0] - radius >= min[0]
                && p[0] + radius <= max[0]
                && p[1] - radius >= min[1]
                && p[1] + radius <= max[1]
        }),
        ChannelRegion::Polyline { points, width_mm } => {
            let r = width_mm / 2.0 - radius;
            r >= 0.0
                && points.windows(2).any(|s| {
                    point_segment_distance(a, s[0], s[1]) <= r
                        && point_segment_distance(b, s[0], s[1]) <= r
                })
        }
    }
}
pub fn point_segment_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let den = d[0] * d[0] + d[1] * d[1];
    let t = if den == 0.0 {
        0.0
    } else {
        ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / den
    }
    .clamp(0.0, 1.0);
    ((p[0] - a[0] - t * d[0]).powi(2) + (p[1] - a[1] - t * d[1]).powi(2)).sqrt()
}

fn checked_size(value: f64) -> Result<i32, String> {
    let v = value.ceil();
    if !v.is_finite() || v < 0.0 || v > 10_000_000.0 {
        return Err("width/clearance exceeds safe engine range".into());
    }
    Ok(v as i32)
}
fn source_protected_copper_counts(board: &RoutingBoard) -> Result<BTreeMap<String, usize>, String> {
    let mut counts = BTreeMap::new();
    for key in board.item_repository().iter() {
        let item = board.item(key);
        if (item.is_trace() || item.is_via())
            && matches!(
                item.fixed_state(),
                FixedState::UserFixed | FixedState::SystemFixed
            )
        {
            *counts
                .entry(source_copper_signature(board, item)?)
                .or_default() += 1;
        }
    }
    Ok(counts)
}
fn source_copper_signature(
    board: &RoutingBoard,
    item: &fr_engine::board::Item,
) -> Result<String, String> {
    let nets: Vec<_> = item
        .net_numbers()
        .iter()
        .map(|&net_no| {
            board
                .rules
                .nets
                .get(net_no)
                .map(|net| net.name.clone())
                .ok_or_else(|| format!("routing copper references unknown net {net_no:?}"))
        })
        .collect::<Result<_, _>>()?;
    let layers: Vec<_> = (item.first_layer(board)..=item.last_layer(board))
        .map(|layer| board.layer_structure.layers[layer as usize].name.as_str())
        .collect();
    let geometry = if item.is_trace() {
        format!(
            "trace:{}:{:?}",
            canonical_polyline_key(&item.trace().polyline().corners()),
            item.trace().half_width()
        )
    } else {
        format!(
            "via:{}:{}",
            via_physical_signature(board, item, None)?,
            canonical_point_key(&item.center(board))
        )
    };
    Ok(format!(
        "{:?}|{nets:?}|{layers:?}|{geometry}|{:?}|{:?}",
        item.board_item_type(),
        item.fixed_state(),
        item.clearance_class()
    ))
}
fn session_item_signature(
    board: &RoutingBoard,
    item: &fr_engine::board::Item,
    authorized_drill_upgrade: Option<u64>,
) -> String {
    let nets: Vec<_> = item
        .net_numbers()
        .iter()
        .map(|&net_no| {
            board
                .rules
                .nets
                .get(net_no)
                .map(|net| net.name.clone())
                .unwrap_or_else(|| format!("unknown:{net_no:?}"))
        })
        .collect();
    let layers: Vec<_> = (item.first_layer(board)..=item.last_layer(board))
        .map(|layer| board.layer_structure.layers[layer as usize].name.as_str())
        .collect();
    let geometry = if item.is_trace() {
        format!(
            "trace:{}:{:?}",
            canonical_polyline_key(&item.trace().polyline().corners()),
            item.trace().half_width()
        )
    } else if item.is_via() {
        format!(
            "via:{}:{}",
            via_physical_signature(board, item, authorized_drill_upgrade)
                .unwrap_or_else(|_| "invalid-via".into()),
            canonical_point_key(&item.center(board))
        )
    } else {
        format!("{:?}", board.item_tile_shapes(item))
    };
    format!(
        "{:?}|{nets:?}|{layers:?}|{geometry}|{:?}|{:?}",
        item.board_item_type(),
        item.fixed_state(),
        item.clearance_class()
    )
}

fn stable_pin_identity(board: &RoutingBoard, item: &fr_engine::board::Item) -> Option<String> {
    let ItemKind::Pin(pin) = &item.kind else {
        return None;
    };
    let component_no = item.component_no();
    if component_no <= 0 || component_no > board.components.count() {
        return None;
    }
    let component = board.components.get(component_no);
    let package = board.library.packages.get(component.get_package());
    Some(format!("{}-{}", component.name, package.get_pin(pin.pin_index)?.name))
}

fn matching_baseline_pin(
    current: &BoardAdapter,
    before: &BoardAdapter,
    current_key: ItemKey,
    before_items: &BTreeMap<i32, ItemKey>,
) -> Option<ItemKey> {
    let current_item = current.board.item(current_key);
    if !current_item.is_pin() || !current_item.is_on_board() {
        return None;
    }
    let identity = stable_pin_identity(&current.board, current_item)?;
    let signature = session_item_signature(&current.board, current_item, None);
    let unchanged = |before_key: ItemKey| {
        let before_item = before.board.item(before_key);
        before_item.is_pin()
            && before_item.is_on_board()
            && before_item.center(&before.board) == current_item.center(&current.board)
            && stable_pin_identity(&before.board, before_item).as_deref() == Some(identity.as_str())
            && session_item_signature(&before.board, before_item, None) == signature
    };

    if let Some(before_key) = before_items.get(&current_item.id().0).copied() {
        if unchanged(before_key) {
            return Some(before_key);
        }
    }

    let mut matching = None;
    for before_key in before.board.get_pins() {
        let item = before.board.item(before_key);
        if stable_pin_identity(&before.board, item).as_deref() != Some(identity.as_str()) {
            continue;
        }
        if !unchanged(before_key) {
            continue;
        }
        if matching.replace(before_key).is_some() {
            return None;
        }
    }
    matching
}

fn other_pin_key_is_unchanged(
    current: &BoardAdapter,
    before: &BoardAdapter,
    current_key: ItemKey,
    before_items: &BTreeMap<i32, ItemKey>,
) -> bool {
    matching_baseline_pin(current, before, current_key, before_items).is_some()
}

fn via_physical_signature(
    board: &RoutingBoard,
    item: &fr_engine::board::Item,
    authorized_drill_upgrade: Option<u64>,
) -> Result<String, String> {
    let via = item.as_via().ok_or("routing item is not a via")?;
    let padstack = board
        .library
        .padstacks
        .get(via.padstack_no())
        .ok_or_else(|| format!("via references unknown padstack {}", via.padstack_no()))?;
    let copper_shapes: Vec<_> = (0..padstack.board_layer_count())
        .map(|layer| padstack.get_shape(layer).map(|shape| format!("{shape:?}")))
        .collect();
    let explicit_drill = padstack
        .get_explicit_drill_radius()
        .map(f64::to_bits)
        .or(authorized_drill_upgrade);
    Ok(format!(
        "{}|copper={copper_shapes:?}|explicit_drill_radius={explicit_drill:?}",
        padstack.name,
    ))
}
fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

/// Full-session materialization is distinct from the upstream append-style SES import.
/// Copper classes come from the source DSN; SES carries geometry and fixed flags only.
fn replay_full_session(
    board: &mut RoutingBoard,
    src: &[u8],
    authorized_vias: &BTreeMap<(NetNo, fr_engine::ids::PadstackNo), fr_engine::rules::ViaInfoId>,
) -> Result<(), String> {
    use fr_dsn::sexpr::{List, Sexpr};
    let expressions = fr_dsn::sexpr::parse(src).map_err(|e| format!("SES parse: {e}"))?;
    if expressions.len() != 1 {
        return Err("SES requires one complete session root".into());
    }
    let root = expressions[0]
        .as_list()
        .filter(|l| l.is("session"))
        .ok_or("SES missing session root")?;
    let routes = root.find("routes").ok_or("SES missing routes")?;
    let network = routes
        .find("network_out")
        .ok_or("SES missing complete network_out")?;
    let resolution = routes
        .find("resolution")
        .ok_or("SES missing routing resolution")?;
    let unit = resolution
        .args()
        .first()
        .and_then(Sexpr::as_atom)
        .ok_or("SES resolution unit missing")?;
    let scale = resolution
        .args()
        .get(1)
        .and_then(Sexpr::as_atom)
        .and_then(|a| a.as_num())
        .ok_or("SES resolution scale missing")?;
    if !unit
        .text
        .eq_ignore_ascii_case(&board.communication.unit.to_string())
        || scale != board.communication.resolution as f64
    {
        return Err("SES resolution differs from source DSN; conversion unsupported".into());
    }
    let den = board.communication.coordinate_transform.dsn_to_board(1.0)
        / board.communication.resolution as f64;
    if !den.is_finite() || den <= 0.0 {
        return Err("invalid SES coordinate conversion".into());
    }
    fn number(list: &List, index: usize) -> Result<f64, String> {
        let x = list
            .args()
            .get(index)
            .and_then(Sexpr::as_atom)
            .and_then(|a| a.as_num())
            .ok_or("invalid SES coordinate")?;
        if !x.is_finite() {
            return Err("nonfinite SES coordinate".into());
        }
        Ok(x)
    }
    fn coordinate(value: f64, den: f64) -> Result<i32, String> {
        let v = (value / den + 0.5).floor();
        if !v.is_finite() || v.abs() > 100_000_000.0 {
            return Err("SES coordinate outside safe engine range".into());
        }
        Ok(v as i32)
    }
    // Explicit candidates authorize this replay only. Serialized library copper
    // must match their independently registered physical dimensions and span.
    let mut validated_padstacks = BTreeSet::new();
    if let Some(library) = routes.find("library_out") {
        for definition in library.sublists().filter(|l| l.is("padstack")) {
            let name = definition
                .args()
                .first()
                .and_then(Sexpr::as_atom)
                .ok_or("SES library padstack name missing")?;
            let Some(pad) = board.library.padstacks.get_by_name(&name.text) else {
                continue;
            };
            if !authorized_vias.keys().any(|(_, id)| *id == pad.id) {
                continue;
            }
            if !validated_padstacks.insert(pad.id) {
                return Err("duplicate authorized SES library padstack".into());
            }
            let mut declared_layers = BTreeSet::new();
            for shape in definition.sublists().filter(|l| l.is("shape")) {
                if shape.args().len() != 1 {
                    return Err("unsupported authorized SES via shape".into());
                }
                let circle = shape
                    .args()
                    .first()
                    .and_then(Sexpr::as_list)
                    .filter(|l| l.is("circle"))
                    .ok_or("authorized SES via must have circular copper")?;
                if circle.args().len() != 4 {
                    return Err("authorized SES via circle needs diameter and center".into());
                }
                let layer_name = circle
                    .args()
                    .first()
                    .and_then(Sexpr::as_atom)
                    .ok_or("SES via copper layer missing")?;
                let layer = board.layer_structure.get_no(&layer_name.text);
                if layer < 0 || !declared_layers.insert(layer) {
                    return Err("invalid or duplicate SES via copper layer".into());
                }
                let expected = pad
                    .get_shape(layer)
                    .ok_or("SES via copper extends outside authorized span")?;
                let diameter = coordinate(number(circle, 1)?, den)?;
                if (diameter as f64 - expected.max_width()).abs() > 1.0
                    || coordinate(number(circle, 2)?, den)?.abs() > 1
                    || coordinate(number(circle, 3)?, den)?.abs() > 1
                {
                    return Err("SES library copper differs from authorized physical via".into());
                }
            }
            let expected_layers: BTreeSet<_> = (pad.from_layer()..=pad.to_layer())
                .filter(|l| pad.get_shape(*l).is_some())
                .collect();
            if declared_layers != expected_layers {
                return Err(
                    "SES library via copper layer span differs from authorized candidate".into(),
                );
            }
        }
    }
    // SES placement is part of the authoritative session state just like route copper. Apply
    // translations before route materialization so pins, component poses, and emitted copper
    // all share the same coordinates. This router currently produces translations only; refuse
    // an unimplemented side/rotation edit rather than silently replaying a partial placement.
    if let Some(placement) = root.find("placement") {
        let den = board.communication.coordinate_transform.dsn_to_board(1.0)
            / board.communication.resolution as f64;
        for package in placement
            .args()
            .iter()
            .filter_map(Sexpr::as_list)
            .filter(|l| l.is("component"))
        {
            for place in package
                .args()
                .iter()
                .filter_map(Sexpr::as_list)
                .filter(|l| l.is("place"))
            {
                let atom = |i: usize| {
                    place
                        .args()
                        .get(i)
                        .and_then(Sexpr::as_atom)
                        .map(|a| a.text.as_str())
                };
                let reference = atom(0).ok_or("SES placement reference missing")?;
                let x = place
                    .args()
                    .get(1)
                    .and_then(Sexpr::as_atom)
                    .and_then(|a| a.as_num())
                    .ok_or("SES placement x missing")?;
                let y = place
                    .args()
                    .get(2)
                    .and_then(Sexpr::as_atom)
                    .and_then(|a| a.as_num())
                    .ok_or("SES placement y missing")?;
                let side = atom(3).ok_or("SES placement side missing")?;
                let rotation = place
                    .args()
                    .get(4)
                    .and_then(Sexpr::as_atom)
                    .and_then(|a| a.as_num())
                    .ok_or("SES placement rotation missing")?;
                let id = board
                    .components
                    .get_by_name(reference)
                    .ok_or_else(|| {
                        format!("SES placement references unknown component {reference}")
                    })?
                    .id;
                let component = board.components.get(id);
                let front = side.eq_ignore_ascii_case("front");
                if component.placed_on_front() != front
                    || (component.get_rotation_in_degree() - rotation).abs() > 1e-6
                {
                    return Err(format!("SES placement for {reference} changes side or rotation; only translation replay is supported"));
                }
                let loc = component
                    .get_location()
                    .ok_or_else(|| format!("SES placement component {reference} is unplaced"))?;
                let target = IntPoint::new(coordinate(x, den)?, coordinate(y, den)?);
                let origin = loc.to_float();
                let dx = target.x - origin.x.round() as i32;
                let dy = target.y - origin.y.round() as i32;
                if dx == 0 && dy == 0 {
                    continue;
                }
                if component.position_fixed {
                    return Err(format!("SES placement moves locked component {reference}"));
                }
                let vector = fr_geom::Vector::get_instance(dx, dy);
                let members: Vec<_> = board
                    .get_items()
                    .into_iter()
                    .filter(|key| board.item(*key).component_no() == id)
                    .collect();
                for key in members {
                    board.translate_item(key, &vector);
                }
                board.components_mut().move_component(id, &vector);
            }
        }
    }
    fn trace_signature(net: i32, layer: i32, half: i32, poly: &Polyline) -> String {
        format!(
            "trace:{net}:{layer}:{half}:{}",
            canonical_polyline_key(&poly.corners())
        )
    }
    fn via_signature(net: i32, pad: i32, point: &Point) -> String {
        format!("via:{net}:{pad}:{}", canonical_point_key(point))
    }
    let mut source_attributes: BTreeMap<String, (i32, FixedState)> = BTreeMap::new();
    let mut purge = Vec::new();
    for key in board.item_repository().iter() {
        let item = board.item(key);
        if !item.is_trace() && !item.is_via() {
            continue;
        }
        if item.net_numbers().len() != 1 {
            return Err("multi-net source copper has ambiguous SES rule inheritance".into());
        }
        let net = item.net_numbers()[0];
        let signature = if item.is_trace() {
            let t = item.trace();
            trace_signature(net, t.layer(), t.half_width(), t.polyline())
        } else {
            via_signature(
                net,
                item.as_via().unwrap().padstack_no(),
                &item.center(board),
            )
        };
        let attributes = (item.clearance_class(), item.fixed_state());
        if source_attributes
            .insert(signature, attributes)
            .is_some_and(|old| old != attributes)
        {
            return Err(
                "coincident source copper has ambiguous class/fixed-state inheritance".into(),
            );
        }
        if item.fixed_state() != FixedState::SystemFixed {
            purge.push(key);
        }
    }
    // Parse the entire replacement before deleting anything from this independent board.
    enum Record {
        Trace {
            net: i32,
            layer: i32,
            half: i32,
            poly: Polyline,
            class: i32,
            fixed: FixedState,
        },
        Via {
            net: i32,
            pad: i32,
            point: Point,
            class: i32,
            fixed: FixedState,
            attach: bool,
        },
    }
    fn canonical(expr: &Sexpr) -> String {
        match expr {
            Sexpr::Atom(a) => format!("a{}:{}", a.text.len(), a.text),
            Sexpr::List(l) => format!(
                "({})",
                l.items.iter().map(canonical).collect::<Vec<_>>().join("")
            ),
        }
    }
    let source_ses = fr_io::ses_writer::ses_bytes(board, "source_area_reference.dsn");
    let source_ast =
        fr_dsn::sexpr::parse(&source_ses).map_err(|e| format!("native source SES parse: {e}"))?;
    let source_network = source_ast
        .first()
        .and_then(Sexpr::as_list)
        .and_then(|l| l.find("routes"))
        .and_then(|l| l.find("network_out"))
        .ok_or("native source SES network missing")?;
    let mut source_areas: BTreeMap<(String, String), usize> = BTreeMap::new();
    for net_scope in source_network.find_all("net") {
        let name = net_scope
            .args()
            .first()
            .and_then(Sexpr::as_atom)
            .ok_or("native source SES net name missing")?;
        for wire in net_scope
            .find_all("wire")
            .filter(|wire| wire.find("path").is_none())
        {
            *source_areas
                .entry((name.text.clone(), canonical(&Sexpr::List(wire.clone()))))
                .or_default() += 1;
        }
    }
    let mut records = Vec::new();
    for net_scope in network.find_all("net") {
        let name = net_scope
            .args()
            .first()
            .and_then(Sexpr::as_atom)
            .ok_or("SES net name missing")?;
        let nets: Vec<_> = board
            .rules
            .nets
            .iter()
            .filter(|n| n.name == name.text)
            .collect();
        if nets.len() != 1 {
            return Err(format!(
                "SES net '{}' not uniquely declared in source DSN",
                name.text
            ));
        }
        let net = nets[0].net_number;
        let net_class = board.rules.net_class_of(net);
        for record in net_scope.sublists() {
            let explicit_fixed = record
                .find("type")
                .and_then(|l| l.args().first())
                .and_then(Sexpr::as_atom)
                .map(|a| a.text.as_str());
            let declared = match explicit_fixed {
                None | Some("normal") => None,
                Some("protect") => Some(FixedState::UserFixed),
                Some("fix") => Some(FixedState::SystemFixed),
                _ => return Err("unsupported SES fixed-state type".into()),
            };
            if record.is("wire") {
                let Some(path) = record.find("path") else {
                    let key = (name.text.clone(), canonical(&Sexpr::List(record.clone())));
                    let count = source_areas
                        .get_mut(&key)
                        .filter(|count| **count > 0)
                        .ok_or(
                        "Unsupported SES area or pathless wire differs from immutable source DSN",
                    )?;
                    *count -= 1;
                    continue;
                };
                let layer_name = path
                    .args()
                    .first()
                    .and_then(Sexpr::as_atom)
                    .ok_or("SES trace layer missing")?;
                let layer = board.layer_structure.get_no(&layer_name.text);
                if layer < 0 {
                    return Err("SES trace layer unknown".into());
                }
                let half = coordinate(number(path, 1)? / 2.0, den)?;
                if half <= 0 {
                    return Err("SES trace width invalid".into());
                }
                let values = path.args().len();
                if values < 6 || (values - 2) % 2 != 0 {
                    return Err("SES path coordinate count invalid".into());
                }
                let mut points = Vec::new();
                for i in (2..values).step_by(2) {
                    points.push(Point::Int(IntPoint::new(
                        coordinate(number(path, i)?, den)?,
                        coordinate(number(path, i + 1)?, den)?,
                    )));
                }
                let poly = Polyline::from_points(&points);
                let source = source_attributes.get(&trace_signature(net, layer, half, &poly));
                let class = source
                    .map(|x| x.0)
                    .unwrap_or_else(|| net_class.get_trace_clearance_class());
                let fixed =
                    declared.unwrap_or_else(|| source.map(|x| x.1).unwrap_or(FixedState::Unfixed));
                records.push(Record::Trace {
                    net,
                    layer,
                    half,
                    poly,
                    class,
                    fixed,
                });
            } else if record.is("via") {
                let pad_name = record
                    .args()
                    .first()
                    .and_then(Sexpr::as_atom)
                    .ok_or("SES via padstack missing")?;
                let pad = board
                    .library
                    .padstacks
                    .get_by_name(&pad_name.text)
                    .ok_or("SES via padstack unknown in source DSN")?;
                let point = Point::Int(IntPoint::new(
                    coordinate(number(record, 1)?, den)?,
                    coordinate(number(record, 2)?, den)?,
                ));
                let source = source_attributes.get(&via_signature(net, pad.id, &point));
                let via_rule = net_class
                    .get_via_rule()
                    .or_else(|| board.rules.via_rules.get_first());
                if authorized_vias.contains_key(&(net, pad.id))
                    && !validated_padstacks.contains(&pad.id)
                {
                    return Err(
                        "new SES via is missing its validated library_out definition".into(),
                    );
                }
                let info = via_rule
                    .and_then(|r| {
                        board.rules.via_rules[r]
                            .vias()
                            .iter()
                            .copied()
                            .find(|&id| board.rules.via_infos[id].get_padstack() == pad.id)
                    })
                    .or_else(|| authorized_vias.get(&(net, pad.id)).copied())
                    .map(|id| &board.rules.via_infos[id]);
                let class = if let Some(source) = source {
                    source.0
                } else {
                    info.ok_or("new SES via is not authorized by source net class")?
                        .get_clearance_class_index()
                };
                let fixed =
                    declared.unwrap_or_else(|| source.map(|x| x.1).unwrap_or(FixedState::Unfixed));
                records.push(Record::Via {
                    net,
                    pad: pad.id,
                    point,
                    class,
                    fixed,
                    attach: info
                        .map(|v| v.attach_smd_allowed())
                        .unwrap_or(pad.attach_allowed),
                });
            } else {
                return Err("unsupported SES network routing record".into());
            }
        }
    }
    if !purge.is_empty() && records.is_empty() {
        return Err("empty SES cannot replace nonempty source routing".into());
    }
    // SesWriter::write_net excludes SystemFixed only; all other trace/via entities are full replacements.
    for key in purge {
        board.item_mut(key).unfix();
        board.remove_item(key);
    }
    // Insert vias before traces: preserve the exported polyline partition without a second splitting/normalization pass.
    for record in &records {
        if let Record::Via {
            net,
            pad,
            point,
            class,
            fixed,
            attach,
        } = record
        {
            board.insert_via(*pad, point.clone(), &[*net], *class, *fixed, *attach);
        }
    }
    for record in records {
        if let Record::Trace {
            net,
            layer,
            half,
            poly,
            class,
            fixed,
        } = record
        {
            board
                .insert_trace_without_cleaning(poly, layer, half, &[net], class, fixed)
                .ok_or("SES trace materialization failed")?;
        }
    }
    board.mark_stitching_vias();
    Ok(())
}
