//! Export a clean DSN board as the smallest text PCBData input accepted by
//! the original AWX parser. This is a format bridge only; it does not route.
use fr_agent::board_adapter::BoardAdapter;
use fr_engine::board::ItemKey;
use fr_geom::Shape;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{collections::BTreeMap, env, fmt::Write as _, fs, path::PathBuf};

#[path = "../src/awx/native_pad_raster.rs"]
mod native_pad_raster;

fn esc(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

fn mm(v: f64) -> String {
    format!("{v:.6}")
}

fn pt(x: f64, y: f64) -> String {
    format!("(xy {} {})", mm(x), mm(y))
}

fn kicad_layer(name: &str) -> String {
    match name {
        "TopLayer" => "F.Cu".into(),
        "Inner1" => "In1.Cu".into(),
        "Inner2" => "In2.Cu".into(),
        "BottomLayer" => "B.Cu".into(),
        _ => name.to_string(),
    }
}

fn points(shape: &Shape, upm: f64) -> Vec<[f64; 2]> {
    shape
        .corner_approx_arr()
        .iter()
        .map(|p| [p.x / upm, p.y / upm])
        .collect()
}

fn identity(board: &BoardAdapter, key: ItemKey) -> (String, String, String, f64, f64, f64, bool) {
    let item = board.board.item(key);
    let pin = item.as_pin().expect("pin key");
    let component = board.board.components.get(item.component_no());
    let package = board.board.library.packages.get(component.get_package());
    let pin_name = package
        .get_pin(pin.pin_index)
        .map(|p| p.name.clone())
        .unwrap_or_default();
    let (x, y) = component
        .get_location()
        .map(|p| {
            let f = p.to_float();
            (f.x / board.units_per_mm, f.y / board.units_per_mm)
        })
        .unwrap_or_else(|| {
            let f = item.center(&board.board).to_float();
            (f.x / board.units_per_mm, f.y / board.units_per_mm)
        });
    (
        component.name.clone(),
        pin_name,
        package.name.clone(),
        x,
        y,
        component.get_rotation_in_degree(),
        component.placed_on_front(),
    )
}

fn pad_text(
    board: &BoardAdapter,
    key: ItemKey,
    upm: f64,
    net_ids: &BTreeMap<i32, String>,
) -> (String, Value) {
    let b = &board.board;
    let item = b.item(key);
    let (reference, pin_name, _package, fx, fy, frot, _front) = identity(board, key);
    let center = item.center(b).to_float();
    let (cx, cy) = (center.x / upm, center.y / upm);
    let mut copper = Vec::new();
    let mut layer_names = Vec::new();
    for layer in item.first_layer(b)..=item.last_layer(b) {
        let l = &b.layer_structure.layers[layer as usize];
        if !l.is_signal {
            continue;
        }
        if let Some(shape) = item.drill_shape_on_layer(b, layer) {
            layer_names.push(kicad_layer(&l.name));
            copper.push((l.name.clone(), shape));
        }
    }

    // The parser's Pad model has one copper outline shared by its layer list.
    // Keep per-layer source geometry in the scene as the lossless record; encode
    // their union into parser-supported custom primitives when layer shapes differ.
    let mut layer_shapes = Vec::new();
    for (layer, shape) in &copper {
        let p = points(shape, upm);
        let kind = match shape {
            Shape::Circle(_) => "circle",
            _ if native_pad_raster::source_rectangle(shape, upm).is_some() => "rect",
            _ => "custom",
        };
        layer_shapes.push(json!({"layer":layer,"kind":kind,"points_mm":p}));
    }
    let same_layers = copper.first().map_or(true, |(_, first)| {
        copper.iter().all(|(_, s)| shape_equal(first, s, upm))
    });
    let representative = if same_layers { copper.first() } else { None };
    let (pad_kind, pad_shape, shape_args) = if let Some((_, representative)) = representative {
        match representative {
            Shape::Circle(c)
                if (c.center.x as f64 / upm - cx).abs() < 1e-6
                    && (c.center.y as f64 / upm - cy).abs() < 1e-6 =>
            {
                (
                    "smd",
                    "circle",
                    format!(
                        "(size {} {})",
                        mm(c.radius as f64 * 2.0 / upm),
                        mm(c.radius as f64 * 2.0 / upm)
                    ),
                )
            }
            shape if native_pad_raster::source_rectangle(shape, upm).is_some() => {
                let r = native_pad_raster::source_rectangle(shape, upm).unwrap();
                let w = r.half_width_mm * 2.0;
                let h = r.half_height_mm * 2.0;
                if (r.center_x_mm - cx).abs() > 1e-6
                    || (r.center_y_mm - cy).abs() > 1e-6
                    || r.rotation_deg.abs() > 1e-6
                {
                    ("smd", "custom", "(size 0.010000 0.010000)".to_string())
                } else if r.corner_radius_mm > 0.0 && r.corner_radius_mm <= w.min(h) / 2.0 {
                    (
                        "smd",
                        "roundrect",
                        format!(
                            "(size {} {}) (roundrect_rratio {})",
                            mm(w),
                            mm(h),
                            mm(r.corner_radius_mm / w.min(h))
                        ),
                    )
                } else {
                    ("smd", "rect", format!("(size {} {})", mm(w), mm(h)))
                }
            }
            _ => ("smd", "custom", "(size 0.010000 0.010000)".to_string()),
        }
    } else {
        (
            if layer_names.len() > 1 {
                "connect"
            } else {
                "smd"
            },
            "custom",
            "(size 0.010000 0.010000)".to_string(),
        )
    };

    let explicit_drill_mm = item
        .padstack(b)
        .and_then(|s| s.get_explicit_drill_radius())
        .map(|r| 2.0 * r / upm);
    let (pad_kind, drill) = match explicit_drill_mm {
        Some(d) if d > 0.0 => ("thru_hole", format!(" (drill {})", mm(d))),
        _ if layer_names.len() > 1 => ("connect", String::new()),
        _ => (pad_kind, String::new()),
    };

    // Use the true pad center as the anchor and absolute board coordinates in
    // custom primitives. This avoids inventing a local footprint transform.
    let theta = frot.to_radians();
    let dx = cx - fx;
    let dy = cy - fy;
    let local_x = theta.cos() * dx - theta.sin() * dy;
    let local_y = theta.sin() * dx + theta.cos() * dy;
    let mut out = format!(
        "      (pad \"{}\" {} {} (at {} {} 0) {} (layers {}){}",
        esc(&pin_name),
        pad_kind,
        pad_shape,
        mm(local_x),
        mm(local_y),
        shape_args,
        layer_names
            .iter()
            .map(|l| format!("\"{}\"", esc(l)))
            .collect::<Vec<_>>()
            .join(" "),
        drill
    );
    if !matches!(pad_shape, "circle" | "rect" | "roundrect") {
        out.push_str(" (primitives");
        for (_layer, shape) in &copper {
            let poly = points(shape, upm);
            if poly.len() >= 3 {
                out.push_str(" (gr_poly (pts");
                for p in poly {
                    out.push(' ');
                    out.push_str(&pt(p[0] - cx, p[1] - cy));
                }
                out.push_str(" ) (width 0) (fill yes))");
            }
        }
        out.push(')');
    }
    let net_no = item.net_numbers().first().copied().unwrap_or(0);
    if net_no != 0 {
        if let Some(name) = net_ids.get(&net_no) {
            write!(out, " (net {} \"{}\")", net_no, esc(name)).unwrap();
        }
    }
    out.push(')');

    let membership = if net_no == 0 {
        Value::Null
    } else {
        json!(net_ids.get(&net_no))
    };
    let data = json!({
        "reference":reference,"pin":pin_name,"net_id":if net_no==0 { Value::Null } else { json!(net_no) },
        "net_name":membership,"center_mm":[cx,cy],"source_drill_mm":explicit_drill_mm,
        "layers":layer_names,"layer_shapes":layer_shapes,"layers_differ":!same_layers
    });
    (out, data)
}

fn shape_equal(a: &Shape, b: &Shape, upm: f64) -> bool {
    match (a, b) {
        (Shape::Circle(x), Shape::Circle(y)) => {
            (x.center.x - y.center.x).abs() <= 1
                && (x.center.y - y.center.y).abs() <= 1
                && (x.radius - y.radius).abs() <= 1
        }
        _ => {
            let ap = points(a, upm);
            let bp = points(b, upm);
            ap.len() == bp.len()
                && ap
                    .iter()
                    .zip(bp.iter())
                    .all(|(x, y)| (x[0] - y[0]).abs() < 1e-6 && (x[1] - y[1]).abs() < 1e-6)
        }
    }
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    if args.len() != 2 {
        return Err("usage: awx_clean_input <board.dsn> <output-directory>".into());
    }
    let source = PathBuf::from(&args[0]);
    let out_dir = PathBuf::from(&args[1]);
    fs::create_dir_all(&out_dir)?;
    let src = fs::read(&source)?;
    let board = BoardAdapter::load(&src).map_err(std::io::Error::other)?;
    let b = &board.board;
    let upm = board.units_per_mm;
    let net_ids: BTreeMap<i32, String> = b
        .rules
        .nets
        .iter()
        .map(|n| (n.net_number, n.name.clone()))
        .collect();

    let mut pcb = String::from("(kicad_pcb (version 20241229) (generator pcbnew)\n  (general (thickness 1.6))\n  (paper \"A4\")\n  (layers\n");
    let mut layer_names = Vec::new();
    for (i, layer) in b.layer_structure.layers.iter().enumerate() {
        let name = kicad_layer(&layer.name);
        layer_names.push(name.clone());
        let ty = if layer.is_signal { "signal" } else { "user" };
        writeln!(pcb, "    ({} \"{}\" {})", i, esc(&name), ty)?;
    }
    let mut next_layer = b.layer_structure.layers.len();
    for (name, ty) in [
        ("F.SilkS", "user"),
        ("B.SilkS", "user"),
        ("F.Mask", "user"),
        ("B.Mask", "user"),
        ("Edge.Cuts", "user"),
    ] {
        if !layer_names.iter().any(|l| l == name) {
            writeln!(pcb, "    ({} \"{}\" {})", next_layer, name, ty)?;
            next_layer += 1;
            layer_names.push(name.to_string());
        }
    }
    pcb.push_str("  )\n  (setup (pad_to_mask_clearance 0))\n  (net 0 \"\")\n");
    for (id, name) in &net_ids {
        writeln!(pcb, "  (net {} \"{}\")", id, esc(name))?;
    }

    let mut pins_scene = Vec::new();
    let pins = b.get_pins();
    let mut footprints: BTreeMap<String, Vec<ItemKey>> = BTreeMap::new();
    for key in &pins {
        let (reference, _, _, _, _, _, _) = identity(&board, *key);
        footprints.entry(reference).or_default().push(*key);
    }
    for (reference, keys) in footprints {
        let (_, _, package, fx, fy, _rotation, front) = identity(&board, keys[0]);
        let side = if front { "F.Cu" } else { "B.Cu" };
        writeln!(
            pcb,
            "  (footprint \"{}\" (layer \"{}\") (at {} {} {})",
            esc(&package),
            side,
            mm(fx),
            mm(fy),
            mm(_rotation)
        )?;
        writeln!(
            pcb,
            "    (property \"Reference\" \"{}\") (property \"Value\" \"{}\")",
            esc(&reference),
            esc(&package)
        )?;
        for key in keys {
            let (pad, data) = pad_text(&board, key, upm, &net_ids);
            pcb.push_str(&pad);
            pcb.push('\n');
            pins_scene.push(data);
        }
        pcb.push_str("  )\n");
    }

    // Edge.Cuts lets the original text parser recover the real board outline.
    let outline = board
        .board_outline_polygons
        .iter()
        .map(|ring| {
            ring.iter()
                .map(|p| [p[0] as f64 / upm, p[1] as f64 / upm])
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    for ring in &outline {
        if ring.len() < 3 {
            continue;
        }
        for i in 0..ring.len() {
            let a = ring[i];
            let z = ring[(i + 1) % ring.len()];
            writeln!(pcb, "  (gr_line (start {} {}) (end {} {}) (stroke (width 0.05) (type default)) (layer \"Edge.Cuts\"))", mm(a[0]),mm(a[1]),mm(z[0]),mm(z[1]))?;
        }
    }

    let mut keepouts = Vec::new();
    for key in b.get_items() {
        let item = b.item(key);
        let Some(area) = item.as_obstacle_area() else {
            continue;
        };
        let layer_area = (item.first_layer(b)..=item.last_layer(b))
            .filter_map(|l| {
                b.layer_structure
                    .layers
                    .get(l as usize)
                    .filter(|x| x.is_signal)
                    .map(|x| kicad_layer(&x.name))
            })
            .collect::<Vec<_>>();
        let polygons = area
            .get_area(b)
            .split_to_convex()
            .unwrap_or_default()
            .iter()
            .map(|s| {
                s.corner_approx_arr()
                    .iter()
                    .map(|p| [p.x / upm, p.y / upm])
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        for poly in &polygons {
            if poly.len() < 3 {
                continue;
            }
            write!(pcb, "  (zone (net 0) (net_name \"\") (layers {}) (keepout (tracks not_allowed) (vias not_allowed) (pads allowed) (copperpour allowed)) (polygon (pts", layer_area.iter().map(|l|format!("\"{}\" ",esc(l))).collect::<String>())?;
            for p in poly {
                write!(pcb, " {}", pt(p[0], p[1]))?;
            }
            pcb.push_str(" )))\n");
        }
        keepouts.push(json!({"native_id":item.id().0,"kind":format!("{:?}",item.board_item_type()),"layers":layer_area,"polygons_mm":polygons,"source_wire_keepout":item.is_wire_obstacle_area(),"source_via_keepout":item.is_via_obstacle_area()}));
    }
    pcb.push_str(")\n");

    let net_pins: BTreeMap<String, Vec<String>> = b
        .rules
        .nets
        .iter()
        .map(|n| {
            let refs = pins
                .iter()
                .filter_map(|&key| {
                    let item = b.item(key);
                    if item.net_numbers().contains(&n.net_number) {
                        let (r, p, _, _, _, _, _) = identity(&board, key);
                        Some(format!("{r}.{p}"))
                    } else {
                        None
                    }
                })
                .collect();
            (n.name.clone(), refs)
        })
        .collect();
    let layers = b.layer_structure.layers.iter().enumerate().map(|(i,l)| json!({"native_index":i,"name":l.name,"kicad_name":kicad_layer(&l.name),"is_signal":l.is_signal})).collect::<Vec<_>>();
    let tracks = b.get_traces().len();
    let vias = b.get_vias().len();
    let initial_incompletes: usize = b
        .rules
        .nets
        .iter()
        .map(|n| {
            let c = b.get_connectable_items(n.net_number);
            fr_engine::drc::NetIncompletes::new(b, n.net_number, &c).count() as usize
        })
        .sum::<usize>();
    let scene = json!({
        "schema_version":1,"source_dsn":source,"source_dsn_sha256":format!("{:x}",Sha256::digest(&src)),"units_per_mm":upm,
        "layers":layers,"nets":b.rules.nets.iter().map(|n|json!({"id":n.net_number,"name":n.name})).collect::<Vec<_>>(),
        "pins":pins_scene,"outline_polygons_mm":outline,"native_obstacle_areas":keepouts,
        "native_base_counts":{"pins":pins.len(),"netless_pins":pins.iter().filter(|&&k|b.item(k).net_numbers().is_empty()).count(),"nets":net_ids.len(),"tracks":tracks,"vias":vias,"initial_incompletes":initial_incompletes,"outline_rings":outline.len(),"obstacle_areas":keepouts.len()},
        "net_pin_memberships":net_pins,
        "different_layer_shape_pads":pins_scene.iter().filter(|p|p["layers_differ"]==true).count(),
        "format_limitations":{"unknown_drill":"omitted from KiCad pad text; parser consequently sees drill=0 and does not infer a cross-layer plated connection","per_layer_pad_geometry":"preserved in input_scene.json; the original parser PCBData Pad holds one polygon set for all pad layers. Identical source layer shapes stay native circle/rectangle primitives; differing source shapes are explicitly counted and their union is present only in the parser input."}
    });
    fs::write(out_dir.join("board.kicad_pcb"), pcb)?;
    fs::write(
        out_dir.join("input_scene.json"),
        serde_json::to_vec_pretty(&scene)?,
    )?;
    fs::write(
        out_dir.join("native_base.json"),
        serde_json::to_vec_pretty(&scene["native_base_counts"])?,
    )?;
    println!("{}", out_dir.display());
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("awx_clean_input: {e}");
        std::process::exit(1);
    }
}
