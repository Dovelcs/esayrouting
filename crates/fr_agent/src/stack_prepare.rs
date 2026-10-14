use crate::board_adapter::BoardAdapter;
use fr_dsn::sexpr::{Sexpr, List, Atom};
use serde::Deserialize;
use std::{fs,path::Path};
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Job { input:String, output_dir:String, budget_ms:u64, grid_step_mm:f64, max_iterations:usize, via_drill_mm:f64, min_annular_ring_mm:f64, hole_to_hole_clearance_mm:f64 }
fn list_mut(e:&mut Sexpr)->Option<&mut List>{if let Sexpr::List(l)=e{Some(l)}else{None}}
fn atom(s:&str)->Sexpr{Sexpr::Atom(Atom{text:s.into(),quoted:false,quoted_len:None,line:0})}
fn write_expr(e:&Sexpr,s:&mut String){match e {Sexpr::Atom(a)=>{if a.quoted {s.push('"');if let Some(n)=a.quoted_len {s.push_str(&a.text[..n].replace('"',"\"\""));s.push('"');s.push_str(&a.text[n..]);}else{s.push_str(&a.text.replace('"',"\"\""));s.push('"');}}else{s.push_str(&a.text)}},Sexpr::List(l)=>{s.push('(');for (i,e) in l.items.iter().enumerate(){if i>0{s.push(' ')}write_expr(e,s)}s.push(')')}}}
fn text(e:&Sexpr)->Option<&str>{e.as_atom().map(|a|a.text.as_str())}
fn child<'a>(l:&'a mut List,name:&str)->Result<&'a mut List,String>{l.items.iter_mut().filter_map(list_mut).find(|x|x.is(name)).ok_or_else(||format!("missing {name}"))}
fn stack(root:&mut List,count:usize)->Result<(),String>{
 let structure=child(root,"structure")?;
 let old:Vec<_>=structure.sublists().filter(|l|l.is("layer")).map(|l|text(&l.items[1]).unwrap_or("").to_owned()).collect();
 if old!=["TopLayer","BottomLayer"] {return Err("expected original TopLayer/BottomLayer stack".into())}
 let bottom=structure.items.iter().position(|e|e.as_list().is_some_and(|l|l.is("layer")&&text(&l.items[1])==Some("BottomLayer"))).unwrap();
 for i in (1..count-1).rev(){let e=fr_dsn::sexpr::parse(format!("(layer Inner{i} (type signal))").as_bytes()).map_err(|e|e.to_string())?.remove(0);structure.items.insert(bottom,e);}
 let library=child(root,"library")?;
 for p in library.items.iter_mut().filter_map(list_mut).filter(|l|l.is("padstack")) {
  let top=p.sublists().filter(|l|l.is("shape")).find(|l|l.items.get(1).and_then(Sexpr::as_list).and_then(|s|s.items.get(1)).and_then(text)==Some("TopLayer")).cloned();
  let bottom=p.sublists().filter(|l|l.is("shape")).find(|l|l.items.get(1).and_then(Sexpr::as_list).and_then(|s|s.items.get(1)).and_then(text)==Some("BottomLayer")).cloned();
  // Native loader treats a padstack spanning both external copper layers as
  // through-hole. Only those shapes extend; single-layer SMD remains external.
  if let (Some(t),Some(_))=(top,bottom) {for i in 1..count-1 {let mut shape=t.clone();let geom=list_mut(&mut shape.items[1]).ok_or("invalid pad shape")?;geom.items[1]=atom(&format!("Inner{i}"));p.items.push(Sexpr::List(shape));}}
 }
 Ok(())
}
fn cross_layer_dsn(count:usize)->String {
 let mut layers=String::from("(layer TopLayer (type signal)) ");
 for i in 1..count-1 {layers.push_str(&format!("(layer Inner{i} (type signal)) "))}
 layers.push_str("(layer BottomLayer (type signal))");
 format!("(PCB cross_layer (resolution mm 1000) (structure {layers} (boundary (rect pcb 0 0 12 10)) (via through) (rule (width 0.25) (clearance 0.15))) (placement (component terminals (place terminals 0 0 front 0))) (library (image terminals (pin top a 2 5) (pin bottom b 10 5)) (padstack top (shape (circle TopLayer 1))) (padstack bottom (shape (circle BottomLayer 1))) (padstack through (shape (circle TopLayer 0.6096)) (shape (circle BottomLayer 0.6096)))) (network (net N (pins terminals-a terminals-b)) (class default N (circuit (use_via through)) (rule (width 0.25) (clearance 0.15)))) (wiring))")
}
fn validate_outputs(input:&Path,output_dir:&Path)->Result<(),String>{
 let source=fs::metadata(input).map_err(|e|e.to_string())?;
 let source_path=fs::canonicalize(input).map_err(|e|e.to_string())?;
 for count in [2,4,6] {
  for name in [format!("board_{count}.dsn"),format!("board_{count}.ses"),format!("board_{count}.jsonl"),format!("case_{count}.toml"),format!("cross_layer_{count}.dsn"),format!("cross_layer_{count}.ses"),format!("cross_layer_{count}.jsonl"),format!("cross_layer_{count}.toml")] {
   let target=output_dir.join(name);
   let metadata=match fs::metadata(&target) {Ok(m)=>m,Err(e) if e.kind()==std::io::ErrorKind::NotFound=>continue,Err(e)=>return Err(e.to_string())};
   let aliases=fs::canonicalize(&target).map_err(|e|e.to_string())?==source_path;
   #[cfg(unix)] {
    use std::os::unix::fs::MetadataExt;
    let aliases=aliases||source.dev()==metadata.dev()&&source.ino()==metadata.ino();
    if aliases {return Err(format!("output {} aliases original input {}",target.display(),input.display()))}
   }
   #[cfg(not(unix))] {
    let _=(&source,&metadata);
    if aliases {return Err(format!("output {} aliases original input {}",target.display(),input.display()))}
   }
  }
 }
 Ok(())
}
pub fn run(path:&Path)
->Result<(),String>{
 let j:Job=toml::from_str(&fs::read_to_string(path).map_err(|e|e.to_string())?).map_err(|e|e.to_string())?;
 validate_outputs(Path::new(&j.input),Path::new(&j.output_dir))?;
 let source=fs::read(&j.input).map_err(|e|e.to_string())?;
 let expressions=fr_dsn::sexpr::parse(&source).map_err(|e|e.to_string())?;
 fs::create_dir_all(&j.output_dir).map_err(|e|e.to_string())?;
 for count in [2,4,6]{let mut tree=expressions.clone();let root=list_mut(tree.first_mut().ok_or("empty DSN")?).ok_or("invalid DSN root")?;
  let wiring=child(root,"wiring")?;wiring.items.retain(|e|!e.as_list().is_some_and(|l|l.is("wire")||l.is("via")));
  stack(root,count)?;
  let mut dsn=String::new();for e in &tree{write_expr(e,&mut dsn);dsn.push('\n')}
  let filename=format!("{}/board_{count}.dsn",j.output_dir);fs::write(&filename,&dsn).map_err(|e|e.to_string())?;
  let raw=BoardAdapter::load_raw(dsn.as_bytes())?;let prepared=BoardAdapter::load(dsn.as_bytes())?;
  let config=format!("input = {:?}\noutput = {:?}\nlog = {:?}\nbudget_ms = {}\ngrid_step_mm = {}\nmax_iterations = {}\nthreads = 1\n# Derived test stack; original DSN drill is unknown. Explicit test rules.\nvia_drill_mm = {}\nmin_annular_ring_mm = {}\nhole_to_hole_clearance_mm = {}\n",filename,format!("{}/board_{count}.ses",j.output_dir),format!("{}/board_{count}.jsonl",j.output_dir),j.budget_ms,j.grid_step_mm,j.max_iterations,j.via_drill_mm,j.min_annular_ring_mm,j.hole_to_hole_clearance_mm);
  fs::write(format!("{}/case_{count}.toml",j.output_dir),&config).map_err(|e|e.to_string())?;
  println!("{}",serde_json::json!({"case":count,"derived_stack":true,"raw_traces":raw.board.get_traces().len(),"raw_vias":raw.board.get_vias().len(),"prepared_traces":prepared.board.get_traces().len(),"prepared_vias":prepared.board.get_vias().len(),"native_nets":prepared.board.rules.nets.iter().count(),"pads":prepared.pads().len(),"stack":prepared.board.layer_structure.layers.iter().map(|l|&l.name).collect::<Vec<_>>()}));
  let cross=cross_layer_dsn(count);
  let mut cross_tree=fr_dsn::sexpr::parse(cross.as_bytes()).map_err(|e|e.to_string())?;
  // Give the physical through-via padstack copper on every test stack layer.
  let cross_root=list_mut(&mut cross_tree[0]).ok_or("invalid cross-layer root")?;
  let cross_library=child(cross_root,"library")?;
  let via=cross_library.items.iter_mut().filter_map(list_mut).find(|l|l.is("padstack")&&text(&l.items[1])==Some("through")).ok_or("missing test via")?;
  for i in 1..count-1 {via.items.push(fr_dsn::sexpr::parse(format!("(shape (circle Inner{i} 0.6096))").as_bytes()).map_err(|e|e.to_string())?.remove(0))}
  let mut smoke=String::new();write_expr(&cross_tree[0],&mut smoke);
  fs::write(format!("{}/cross_layer_{count}.dsn",j.output_dir),smoke).map_err(|e|e.to_string())?;
  let cross_config=config.replace(&format!("board_{count}"),&format!("cross_layer_{count}"));
  fs::write(format!("{}/cross_layer_{count}.toml",j.output_dir),cross_config).map_err(|e|e.to_string())?;
  if raw.board.get_traces()
.len()!=0 || raw.board.get_vias().len()!=0 {return Err("clean DSN contains route copper".into())}
 }
 Ok(())
}
