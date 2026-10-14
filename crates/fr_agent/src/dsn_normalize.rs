use fr_dsn::sexpr::{Atom, List, Sexpr};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Debug, Clone, Serialize)]
pub struct NameMapping {
    pub line: u32,
    pub context: String,
    pub original_atoms: Vec<String>,
    pub normalized_name: String,
    pub transformation: String,
}
#[derive(Debug, Clone, Serialize)]
pub struct NormalizationEvidence {
    pub source_sha256: String,
    pub normalized_sha256: String,
    pub mappings: Vec<NameMapping>,
    pub implicit_nets_from_copper: Vec<String>,
    pub implicit_net_pin_counts: BTreeMap<String, usize>,
    pub unused_class_members: Vec<String>,
    pub original_declared_nets: usize,
    pub normalized_declared_nets: usize,
    pub pads: usize,
    pub source_wires: usize,
    pub source_vias: usize,
    pub geometry_and_rule_atoms_unchanged: bool,
    pub copper_net_assignments_preserved: bool,
    pub connectivity_unverified: bool,
    pub identifier_case_sensitive: bool,
    pub all_source_rule_members_bound: bool,
    pub source_literal_placeholder_classes: Vec<String>,
}
#[derive(Debug)]
pub struct NormalizedDsn {
    pub bytes: Vec<u8>,
    pub evidence: NormalizationEvidence,
}
fn hash(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
fn atoms(l: &List) -> Vec<Atom> {
    l.leading_atoms().cloned().collect()
}
fn named_prefix(a: &[Atom]) -> Result<(String, Option<Atom>), String> {
    if a.is_empty() {
        return Err("empty net name".into());
    }
    let numeric = if a.len() > 1
        && !a.last().unwrap().quoted
        && a.last().unwrap().text.parse::<i32>().is_ok()
    {
        Some(a.last().unwrap().clone())
    } else {
        None
    };
    let end = a.len() - usize::from(numeric.is_some());
    if end == 0 || a[..end].iter().any(|a| a.text.is_empty()) {
        return Err("ambiguous net name".into());
    }
    if a[..end].iter().any(|a| a.quoted) && end != 1 {
        return Err("quoted net name followed by nonnumeric tokens".into());
    }
    Ok((
        a[..end]
            .iter()
            .map(|a| a.text.as_str())
            .collect::<Vec<_>>()
            .join(" "),
        numeric,
    ))
}
fn quoted(text: String, line: u32) -> Sexpr {
    Sexpr::Atom(Atom {
        quoted: true,
        quoted_len: Some(text.len()),
        text,
        line,
    })
}
fn replace_prefix(l: &mut List, prefix: Vec<Sexpr>) {
    let count = l.leading_atoms().count();
    l.items.splice(1..1 + count, prefix);
}
fn walk_lists<'a>(node: &'a Sexpr, found: &mut Vec<&'a List>, head: &str) {
    if let Sexpr::List(l) = node {
        if l.is(head) {
            found.push(l);
        }
        for child in &l.items {
            walk_lists(child, found, head);
        }
    }
}
fn invariant_atoms(node: &Sexpr, out: &mut Vec<String>) {
    match node {
        Sexpr::Atom(a) => out.push(a.text.clone()),
        Sexpr::List(l) => {
            out.push(format!("({}", l.head().unwrap_or("")));
            let skip = if l.is("net") || l.is("class") {
                l.leading_atoms().count() + 1
            } else {
                1
            };
            for child in l.items.iter().skip(skip) {
                invariant_atoms(child, out);
            }
            out.push(")".into());
        }
    }
}
fn render(node: &Sexpr, out: &mut String) {
    match node {
        Sexpr::Atom(a) => {
            if a.quoted {
                let len = a.quoted_len.unwrap_or(a.text.len());
                out.push('"');
                out.push_str(&a.text[..len].replace('"', "\"\""));
                out.push('"');
                out.push_str(&a.text[len..]);
            } else {
                out.push_str(&a.text);
            }
        }
        Sexpr::List(l) => {
            out.push('(');
            for (i, n) in l.items.iter().enumerate() {
                if i > 0 {
                    out.push(' ');
                }
                render(n, out);
            }
            out.push(')');
        }
    }
}
fn class_prefix(a: &[Atom]) -> Result<(String, Vec<String>), String> {
    if a.len() == 1 && a[0].text == "''" {
        // This source's unquoted identifier is literally two apostrophes.
        return Ok(("''".into(), Vec::new()));
    }
    let start = a.iter().position(|a| a.text.starts_with('\''));
    let Some(start) = start else {
        // Standard classes use one class name followed by individually quoted/atomic net names.
        let first = a.first().ok_or("empty class")?;
        if first.text.trim().is_empty() || a[1..].iter().any(|a| a.text.trim().is_empty()) {
            return Err("empty class name or member".into());
        }
        return Ok((
            first.text.clone(),
            a[1..].iter().map(|a| a.text.clone()).collect(),
        ));
    };
    if start == 0 {
        return Err("class name cannot be empty".into());
    }
    let name = a[..start]
        .iter()
        .map(|a| a.text.as_str())
        .collect::<Vec<_>>()
        .join(" ");
    let mut members = Vec::new();
    let mut i = start;
    while i < a.len() {
        if !a[i].text.starts_with('\'') {
            return Err("class member missing opening single quote".into());
        }
        let mut text = a[i].text[1..].to_owned();
        i += 1;
        while !text.ends_with('\'') {
            let next = a
                .get(i)
                .ok_or("unterminated exporter single-quoted class member")?;
            text.push(' ');
            text.push_str(&next.text);
            i += 1;
        }
        text.pop();
        if text.contains('\'') {
            return Err("ambiguous embedded single quote in class member".into());
        }
        if !text.is_empty() {
            members.push(text);
        }
    }
    Ok((name, members))
}
// Cadence SPECCTRA Design Language Reference, version 10.0, PDF page 83
// (case_sensitive): default OFF treats clk/CLK as the same object identifier.
// https://cdn.hackaday.io/files/1666717130852064/specctra.pdf
// The current engine supports OFF only; ON is rejected rather than silently merged.
pub fn normalize(bytes: &[u8]) -> Result<NormalizedDsn, String> {
    let mut roots = fr_dsn::sexpr::parse(bytes).map_err(|e| e.to_string())?;
    if roots.len() != 1 {
        return Err("expected one PCB root".into());
    }
    let root = roots[0].as_list().ok_or("PCB root is not a list")?;
    let case_sensitive = match root
        .find("parser")
        .and_then(|p| p.find("case_sensitive"))
        .and_then(|c| c.leading_atoms().next())
        .map(|a| a.text.to_ascii_lowercase())
    {
        None => false,
        Some(v) if v == "off" => false,
        Some(v) if v == "on" => true,
        Some(_) => return Err("invalid parser case_sensitive flag".into()),
    };
    if case_sensitive {
        return Err(
            "unsupported parser case_sensitive ON: routing engine name lookup is case-insensitive"
                .into(),
        );
    }
    let key = |name: &str| {
        if case_sensitive {
            name.to_owned()
        } else {
            name.to_ascii_lowercase()
        }
    };
    let network = root.find("network").ok_or("missing network")?;
    let mut declared = BTreeSet::new();
    let mut canonical = BTreeMap::new();
    for l in network.find_all("net") {
        let (name, subnet) = named_prefix(&atoms(l))?;
        if !declared.insert(name.clone()) {
            return Err(format!("duplicate original semantic net: {name}"));
        }
        let _ = subnet;
        if canonical.insert(key(&name), name.clone()).is_some() {
            return Err(format!(
                "duplicate semantic identifier under parser case mode: {name}"
            ));
        }
    }
    let original_declared_nets = declared.len();
    let mut net_lists = Vec::new();
    walk_lists(&roots[0], &mut net_lists, "net");
    let mut assignments = Vec::new();
    let mut implicit = BTreeSet::new();
    for l in net_lists {
        let (name, subnet) = named_prefix(&atoms(l))?;
        assignments.push((key(&name), subnet.as_ref().map(|a| a.text.clone())));
        if let std::collections::btree_map::Entry::Vacant(entry) = canonical.entry(key(&name)) {
            entry.insert(name.clone());
            implicit.insert(name);
        }
    }
    let all_names: BTreeSet<_> = declared.union(&implicit).cloned().collect();
    let mut before_classes = Vec::new();
    walk_lists(&roots[0], &mut before_classes, "class");
    let expected_classes: Vec<_> = before_classes
        .iter()
        .map(|l| -> Result<_, String> {
            let (name, members) = class_prefix(&atoms(l))?;
            let members: Vec<String> = members
                .into_iter()
                .map(|m| canonical.get(&key(&m)).cloned().unwrap_or(m))
                .collect();
            let mut rules = Vec::new();
            for child in l.items.iter().skip(l.leading_atoms().count() + 1) {
                invariant_atoms(child, &mut rules);
            }
            Ok((name, members, rules))
        })
        .collect::<Result<_, _>>()?;
    let mut invariant_before = Vec::new();
    invariant_atoms(&roots[0], &mut invariant_before);
    let mut mappings = Vec::new();
    let mut unused = BTreeSet::new();
    fn transform(
        node: &mut Sexpr,
        names: &BTreeMap<String, String>,
        case_sensitive: bool,
        mappings: &mut Vec<NameMapping>,
        unused: &mut BTreeSet<String>,
    ) -> Result<(), String> {
        if let Sexpr::List(l) = node {
            if l.is("net") {
                let original = atoms(l);
                let (source_name, subnet) = named_prefix(&original)?;
                let key = if case_sensitive {
                    source_name.clone()
                } else {
                    source_name.to_ascii_lowercase()
                };
                let name = names
                    .get(&key)
                    .ok_or_else(|| format!("unknown net reference: {source_name}"))?
                    .clone();
                let mut prefix = vec![quoted(name.clone(), l.line)];
                if let Some(s) = subnet {
                    prefix.push(Sexpr::Atom(s));
                }
                if original.len() > 1 || original.first().is_some_and(|a| !a.quoted) {
                    mappings.push(NameMapping {
                        line: l.line,
                        context: "net".into(),
                        original_atoms: original.iter().map(|a| a.text.clone()).collect(),
                        transformation: if source_name != name {
                            "case_canonicalization"
                        } else if original.len() > 1 {
                            "multiword_quoting"
                        } else {
                            "identifier_quoting"
                        }
                        .into(),
                        normalized_name: name,
                    });
                }
                replace_prefix(l, prefix);
            } else if l.is("class") {
                let original = atoms(l);
                let (name, source_members) = class_prefix(&original)?;
                let mut members = Vec::new();
                for member in source_members {
                    let key = if case_sensitive {
                        member.clone()
                    } else {
                        member.to_ascii_lowercase()
                    };
                    if let Some(canonical) = names.get(&key) {
                        members.push(canonical.clone());
                    } else {
                        unused.insert(member.clone());
                        members.push(member);
                    }
                }
                let mut prefix = vec![quoted(name.clone(), l.line)];
                prefix.extend(members.into_iter().map(|s| quoted(s, l.line)));
                mappings.push(NameMapping {
                    line: l.line,
                    context: "class".into(),
                    original_atoms: original.iter().map(|a| a.text.clone()).collect(),
                    normalized_name: name,
                    transformation: "class_name_and_member_quoting".into(),
                });
                replace_prefix(l, prefix);
            }
            for child in &mut l.items {
                transform(child, names, case_sensitive, mappings, unused)?;
            }
        }
        Ok(())
    }
    transform(
        &mut roots[0],
        &canonical,
        case_sensitive,
        &mut mappings,
        &mut unused,
    )?;
    let mut after_classes = Vec::new();
    walk_lists(&roots[0], &mut after_classes, "class");
    let actual_classes: Vec<_> = after_classes
        .iter()
        .map(|l| -> Result<_, String> {
            let (name, members) = class_prefix(&atoms(l))?;
            let mut rules = Vec::new();
            for child in l.items.iter().skip(l.leading_atoms().count() + 1) {
                invariant_atoms(child, &mut rules);
            }
            Ok((name, members, rules))
        })
        .collect::<Result<_, _>>()?;
    if expected_classes != actual_classes {
        return Err("class name/member/rule association changed".into());
    }
    // Source copper labels with no pin declaration retain their own independent net identity.
    let Sexpr::List(root) = &mut roots[0] else {
        unreachable!()
    };
    let network = root
        .items
        .iter_mut()
        .find_map(|n| match n {
            Sexpr::List(l) if l.is("network") => Some(l),
            _ => None,
        })
        .unwrap();
    for name in &implicit {
        network.items.push(Sexpr::List(List {
            line: network.line,
            items: vec![
                Sexpr::Atom(Atom {
                    text: "net".into(),
                    quoted: false,
                    quoted_len: None,
                    line: network.line,
                }),
                quoted(name.clone(), network.line),
                Sexpr::List(List {
                    line: network.line,
                    items: vec![Sexpr::Atom(Atom {
                        text: "pins".into(),
                        quoted: false,
                        quoted_len: None,
                        line: network.line,
                    })],
                }),
            ],
        }));
    }
    // Compare only the original subtree before added implicit declarations.
    let mut invariant_root = roots[0].clone();
    if let Sexpr::List(r) = &mut invariant_root {
        for n in &mut r.items {
            if let Sexpr::List(l) = n {
                if l.is("network") {
                    l.items.truncate(l.items.len() - implicit.len());
                }
            }
        }
    }
    let mut invariant_after = Vec::new();
    invariant_atoms(&invariant_root, &mut invariant_after);
    if invariant_before != invariant_after {
        return Err("geometry/rule invariant changed".into());
    }
    let mut rendered = String::new();
    render(&roots[0], &mut rendered);
    rendered.push('\n');
    let result = rendered.into_bytes();
    let typed = fr_dsn::Dsn::parse(&result).map_err(|e| e.to_string())?;
    let mut after_lists = Vec::new();
    walk_lists(&invariant_root, &mut after_lists, "net");
    let after_assignments: Vec<_> = after_lists
        .iter()
        .map(|l| {
            let (n, s) = named_prefix(&atoms(l)).unwrap();
            (key(&n), s.map(|a| a.text))
        })
        .collect();
    if assignments != after_assignments {
        return Err("copper net identity changed".into());
    }
    if typed
        .network
        .nets
        .iter()
        .map(|n| n.name.clone())
        .collect::<BTreeSet<_>>()
        != all_names
    {
        return Err("normalized net declaration count mismatch".into());
    }
    let parsed_again = fr_dsn::sexpr::parse(&result).map_err(|e| e.to_string())?;
    if parsed_again != roots {
        // line numbers necessarily change; compare semantic atom fingerprints below.
        let mut a = Vec::new();
        let mut b = Vec::new();
        invariant_atoms(&parsed_again[0], &mut a);
        invariant_atoms(&roots[0], &mut b);
        if a != b {
            return Err("serialized geometry/rules changed".into());
        }
    }
    let pads = typed.library.images.iter().map(|i| i.pins.len()).sum();
    let source_literal_placeholder_classes: Vec<String> = typed
        .network
        .classes
        .iter()
        .filter(|c| c.name == "''")
        .map(|c| c.name.clone())
        .collect();
    let all_source_rule_members_bound =
        unused.is_empty() && source_literal_placeholder_classes.is_empty();
    let implicit_net_pin_counts = implicit.iter().map(|n| (n.clone(), 0)).collect();
    Ok(NormalizedDsn {
        bytes: result.clone(),
        evidence: NormalizationEvidence {
            source_sha256: hash(bytes),
            normalized_sha256: hash(&result),
            mappings,
            implicit_nets_from_copper: implicit.into_iter().collect(),
            implicit_net_pin_counts,
            unused_class_members: unused.into_iter().collect(),
            original_declared_nets,
            normalized_declared_nets: typed.network.nets.len(),
            pads,
            source_wires: typed.wiring.wires.len(),
            source_vias: typed.wiring.vias.len(),
            geometry_and_rule_atoms_unchanged: true,
            copper_net_assignments_preserved: true,
            connectivity_unverified: true,
            identifier_case_sensitive: case_sensitive,
            all_source_rule_members_bound,
            source_literal_placeholder_classes,
        },
    })
}

/// Stage both files, sync their data, and publish with atomic no-clobber links.
pub fn write_normalized(
    n: &NormalizedDsn,
    dsn: &std::path::Path,
    mapping: &std::path::Path,
) -> Result<(), String> {
    use std::io::Write;
    fn parent(p: &std::path::Path) -> &std::path::Path {
        p.parent()
            .filter(|p| !p.as_os_str().is_empty())
            .unwrap_or(std::path::Path::new("."))
    }
    let mut d = tempfile::NamedTempFile::new_in(parent(dsn)).map_err(|e| e.to_string())?;
    let mut j = tempfile::NamedTempFile::new_in(parent(mapping)).map_err(|e| e.to_string())?;
    d.write_all(&n.bytes).map_err(|e| e.to_string())?;
    j.write_all(&serde_json::to_vec_pretty(&n.evidence).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    d.as_file().sync_all().map_err(|e| e.to_string())?;
    j.as_file().sync_all().map_err(|e| e.to_string())?;
    // Check all destinations before publication; persist_noclobber remains the race guard.
    for path in [dsn, mapping] {
        if std::fs::symlink_metadata(path).is_ok() {
            return Err(format!("refusing to overwrite {}", path.display()));
        }
    }
    let owned = d.persist_noclobber(dsn).map_err(|e| e.to_string())?;
    if let Err(error) = j.persist_noclobber(mapping) {
        #[cfg(unix)]
        {
            use std::os::unix::fs::MetadataExt;
            let our = owned.metadata().map_err(|e| e.to_string())?;
            if let Ok(now) = std::fs::symlink_metadata(dsn) {
                if now.dev() == our.dev() && now.ino() == our.ino() {
                    std::fs::remove_file(dsn).map_err(|e| e.to_string())?;
                }
            }
        }
        return Err(error.to_string());
    }
    Ok(())
}
