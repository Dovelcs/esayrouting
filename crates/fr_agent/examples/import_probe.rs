//! Read-only diagnosis of flattened pad/via representations; makes no repairs.
use fr_agent::board_adapter::BoardAdapter;
use fr_dsn::sexpr::{List, Sexpr};
fn serialize(node: &Sexpr) -> String {
    match node {
        Sexpr::Atom(a) => {
            if a.quoted {
                format!("\"{}\"", a.text)
            } else {
                a.text.clone()
            }
        }
        Sexpr::List(l) => format!(
            "({})",
            l.items.iter().map(serialize).collect::<Vec<_>>().join(" ")
        ),
    }
}
fn collect<'a>(nodes: &'a [Sexpr], kind: &str, result: &mut Vec<&'a List>) {
    for node in nodes {
        if let Some(l) = node.as_list() {
            if l.is(kind) {
                result.push(l)
            }
            collect(&l.items, kind, result)
        }
    }
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args()
        .nth(1)
        .ok_or("usage: import_probe input.dsn")?;
    let bytes = std::fs::read(path)?;
    let board = BoardAdapter::load_raw(&bytes)?;
    let b = &board.board;
    let pins = b.get_pins();
    let vias = b.get_vias();
    let mut exact = Vec::new();
    let mut unmatched = 0;
    for &pin in &pins {
        let p = b.item(pin);
        if !p.net_numbers().is_empty() {
            continue;
        }
        let found: Vec<_> = vias
            .iter()
            .copied()
            .filter(|&key| {
                let v = b.item(key);
                p.center(b) == v.center(b)
                    && p.first_layer(b) == v.first_layer(b)
                    && p.last_layer(b) == v.last_layer(b)
                    && format!("{:?}", p.drill_shapes(b)) == format!("{:?}", v.drill_shapes(b))
            })
            .collect();
        if found.is_empty() {
            unmatched += 1
        } else {
            exact.push(serde_json::json!({"pin_id":p.id().0,"pin_name":board.pads().iter().find(|pad|pad.key==Some(pin)).map(|pad|pad.pad.clone()),"via_ids":found.iter().map(|&k|b.item(k).id().0).collect::<Vec<_>>(),"center_mm":[p.center(b).to_float().x/board.units_per_mm,p.center(b).to_float().y/board.units_per_mm],"pin_padstack":p.padstack(b).map(|x|x.name.clone()),"via_padstacks":found.iter().map(|&k|b.item(k).padstack(b).map(|x|x.name.clone())).collect::<Vec<_>>(),"via_nets":found.iter().map(|&k|b.item(k).net_numbers().iter().filter_map(|&n|b.rules.nets.get(n).map(|net|net.name.clone())).collect::<Vec<_>>()).collect::<Vec<_>>()}));
        }
    }
    let nodes = fr_dsn::sexpr::parse(&bytes)?;
    let mut resolutions = Vec::new();
    collect(&nodes, "resolution", &mut resolutions);
    let source_to_mm = match resolutions
        .first()
        .and_then(|l| l.leading_atoms().next())
        .map(|a| a.text.as_str())
    {
        Some("mil") => 0.0254,
        Some("mm") => 1.0,
        Some("inch") => 25.4,
        _ => return Err("source unit unsupported by read-only probe".into()),
    };
    let mut source_pins = Vec::new();
    let mut source_vias = Vec::new();
    let mut padstacks = Vec::new();
    collect(&nodes, "pin", &mut source_pins);
    collect(&nodes, "via", &mut source_vias);
    collect(&nodes, "padstack", &mut padstacks);
    let samples:Vec<_>=exact.iter().take(5).map(|row|{let padstack=row["pin_padstack"].as_str().unwrap_or("");let matching_pin=source_pins.iter().find(|p|p.leading_atoms().nth(1).is_some_and(|a|Some(a.text.as_str())==row["pin_name"].as_str()));let via_pad=row["via_padstacks"][0].as_str().unwrap_or("");let matching_via=source_vias.iter().find(|v|{let a:Vec<_>=v.leading_atoms().collect();a.len()>=3&&a[0].text==via_pad&&a[1].text.parse::<f64>().ok().is_some_and(|x|(x*source_to_mm-row["center_mm"][0].as_f64().unwrap()).abs()<0.0001)&&a[2].text.parse::<f64>().ok().is_some_and(|y|(y*source_to_mm-row["center_mm"][1].as_f64().unwrap()).abs()<0.0001)});let ps=padstacks.iter().filter(|p|p.leading_atoms().next().is_some_and(|a|a.text==padstack||a.text==via_pad)).map(|p|serialize(&Sexpr::List((*p).clone()))).collect::<Vec<_>>();serde_json::json!({"native":row,"source_pin":matching_pin.map(|p|serialize(&Sexpr::List((*p).clone()))),"source_via":matching_via.map(|p|serialize(&Sexpr::List((*p).clone()))),"source_padstacks":ps})}).collect();
    println!(
        "{}",
        serde_json::to_string_pretty(
            &serde_json::json!({"pins":pins.len(),"vias":vias.len(),"netless_pins":pins.iter().filter(|&&k|b.item(k).net_numbers().is_empty()).count(),"netless_pins_exact_via_match":exact.len(),"netless_pins_without_exact_via":unmatched,"matches":exact,"samples":samples})
        )?
    );
    Ok(())
}
