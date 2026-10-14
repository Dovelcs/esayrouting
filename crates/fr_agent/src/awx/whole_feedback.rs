//! Deterministic port of the endpoint feedback rules in pinned `awx/whole_feedback.py`.
use super::{
    corridor::Point,
    escape_moves::{self, MoveKind},
    joint_escape::{JointPlan, OptionMove},
    whole_context::WholeContext,
    whole_ends::{EndsFeedback, EndsModel, FeedbackEnd},
    whole_route::WholeLoopOutcome,
    whole_types::HistoryAudit,
};
use std::collections::BTreeSet;

const NAME_TOP: usize = 3;
const LOAD_OK: f64 = 0.5;

fn end_points(ctx: &WholeContext, lane: &str, end: usize) -> Option<Vec<Point>> {
    if let Some(points) = ctx.pair_ends.get(lane) {
        return Some(points[end].to_vec());
    }
    let ends = ctx.ends.get(lane)?;
    Some(vec![if end == 0 { ends.tooth } else { ends.berth }])
}

fn end_box(ctx: &WholeContext, end: usize) -> Option<[f64; 4]> {
    let points: Vec<_> = ctx.lanes.iter().filter_map(|lane| end_points(ctx, lane, end)).flatten().collect();
    if points.is_empty() { return None; }
    Some([
        points.iter().map(|p|p[0]).fold(f64::INFINITY,f64::min),
        points.iter().map(|p|p[1]).fold(f64::INFINITY,f64::min),
        points.iter().map(|p|p[0]).fold(f64::NEG_INFINITY,f64::max),
        points.iter().map(|p|p[1]).fold(f64::NEG_INFINITY,f64::max),
    ])
}

fn near(ctx:&WholeContext, point:Point, end:usize, bounds:[f64;4])->bool {
    let dx=(bounds[0]-point[0]).max(0.).max(point[0]-bounds[2]);
    let dy=(bounds[1]-point[1]).max(0.).max(point[1]-bounds[3]);
    if dx==0. && dy>0. {
        let y0=if point[1]<bounds[1]{bounds[1]}else{bounds[3]};
        let n:usize=ctx.lanes.iter().filter_map(|lane|end_points(ctx,lane,end)).flatten().filter(|p|(p[1]-y0).abs()<1e-3).count();
        dy <= (n as f64+2.)*ctx.cfg.lane_min
    } else {
        dx.hypot(dy) <= 2.*ctx.cfg.via_need()+ctx.cfg.lane_min
    }
}

fn lane_for(ctx:&WholeContext, name:&str)->String {
    ctx.pairs.iter().find_map(|(base,legs)|legs.iter().any(|n|n==name).then(||base.clone())).unwrap_or_else(||name.to_owned())
}

fn lanes_named(ctx:&WholeContext,names:&[String])->Vec<String>{
    let mut seen=BTreeSet::new(); let mut out=Vec::new();
    for name in names {let lane=lane_for(ctx,name);if ctx.ends.contains_key(&lane)&&seen.insert(lane.clone()){out.push(lane);}}
    out
}

fn make_end(ctx:&WholeContext,lane:&str,end:usize,round:usize)->Option<FeedbackEnd>{
    let points=end_points(ctx,lane,end)?;
    let layer=if let Some(pair)=ctx.pair_layers.get(lane){pair[end].clone()}else if end==0{
        let leg=ctx.pairs.get(lane).map(|p|p[0].as_str()).unwrap_or(lane);ctx.tooth_layer.get(leg)?.clone()
    }else{let leg=ctx.pairs.get(lane).map(|p|p[0].as_str()).unwrap_or(lane);ctx.dest_layer.get(leg)?.clone()};
    Some(FeedbackEnd{lane:lane.to_owned(),end,points,layer,times:1,round:Some(round)})
}

fn same_end(a:&FeedbackEnd,b:&FeedbackEnd)->bool{a.lane==b.lane&&a.end==b.end&&a.points==b.points&&a.layer==b.layer}
fn same_pair(a:&[FeedbackEnd;2],b:&[FeedbackEnd;2])->bool{
    (same_end(&a[0],&b[0])&&same_end(&a[1],&b[1]))||(same_end(&a[0],&b[1])&&same_end(&a[1],&b[0]))
}

/// Add an endpoint finding or raise its repeat price once in this outer round.
fn put_avoid(feedback:&mut EndsFeedback,item:FeedbackEnd,round:usize,raised:&mut BTreeSet<(String,usize,String)>){
    let key=(item.lane.clone(),item.end,format!("{}|{}",item.layer,item.points.iter().map(|p|format!("{:.6},{:.6}",p[0],p[1])).collect::<Vec<_>>().join(";")));
    if !raised.insert(key){return;}
    if let Some(old)=feedback.avoid.iter_mut().find(|old|same_end(old,&item)){
        if old.round==Some(round){return;}
        old.times=old.times.max(1).saturating_add(1);old.round=Some(round);
    }else{feedback.avoid.push(item);}
}

fn plan_has_via_in_pad(plan:&JointPlan,ctx:&WholeContext,lane:&str)->bool{
    let contains=|member:&str|plan.chosen.iter().any(|(name,choice)|{
        (name==member||escape_moves::short_name(name)==escape_moves::short_name(member))&&matches!(choice,OptionMove::Escape{movement} if movement.kind==MoveKind::ViaInPad)
    });
    if let Some(pair)=ctx.pairs.get(lane){contains(&pair[0])||contains(&pair[1])}else{contains(lane)}
}

/// Derive endpoint feedback from all whole-route audit rounds and the current fanout result.
/// Repeated endpoint findings are escalated at most once per outer round. Open lanes are named
/// at their failing endpoints; via-in-pad failure at one end widens feedback to its other end.
pub fn next_round(
    ctx:&WholeContext,
    outcome:&WholeLoopOutcome,
    model:&EndsModel,
    previous:&EndsFeedback,
    open:&[String],
    round:usize,
    source_plan:&JointPlan,
    destination_plan:&JointPlan,
)->EndsFeedback{
    let mut next=previous.clone();
    // HotFinding and WholeContext geometry both use the route's canonical FlowFrame.
    // Keep their endpoints in that frame so whole_ends does not apply the board-to-flow
    // mirror/quarter-turn a second time.
    next.points_in_flow_frame=true;
    for (lane,over) in &model.lane_over{next.over.entry(lane.clone()).and_modify(|v|*v=(*v).max(*over)).or_insert(*over);}
    let boxes=[end_box(ctx,0),end_box(ctx,1)];
    let mut hot_rows=BTreeSet::new();let mut raised=BTreeSet::new();
    for history in &outcome.history {for hot in &history.hot {
        let lanes=lanes_named(ctx,&hot.lanes);
        for end in 0..2 {if boxes[end].is_some_and(|b|near(ctx,hot.point,end,b)) {
            for lane in &lanes{hot_rows.insert((lane.clone(),end));}
            if lanes.len()==1 {if let Some(item)=make_end(ctx,&lanes[0],end,round){put_avoid(&mut next,item,round,&mut raised);}}
            else {for i in 0..lanes.len(){for j in i+1..lanes.len(){
                if let (Some(a),Some(b))=(make_end(ctx,&lanes[i],end,round),make_end(ctx,&lanes[j],end,round)){
                    let mut pair=[a,b];pair.sort_by(|a,b|a.lane.cmp(&b.lane));
                    if !next.pairs.iter().any(|old|same_pair(old,&pair)){next.pairs.push(pair);}
                }
            }}}
        }}
    }}
    let mut named=lanes_named(ctx,open);
    if named.is_empty(){
        let mut over:Vec<_>=model.lane_over.iter().filter(|(_,v)|**v>0).map(|(n,v)|(n.clone(),*v)).collect();
        over.sort_by(|a,b|b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
        named=over.into_iter().map(|x|x.0).filter(|n|fresh(ctx,&next,n)).collect();
        if named.is_empty(){let mut loaded:Vec<_>=model.lane_load.iter().filter(|(_,v)|**v>LOAD_OK).map(|(n,v)|(n.clone(),*v)).collect();loaded.sort_by(|a,b|b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));named=loaded.into_iter().map(|x|x.0).filter(|n|fresh(ctx,&next,n)).collect();}
        if named.is_empty(){let mut crossed:Vec<_>=model.lane_x.iter().map(|(n,x)|(n.clone(),*x)).collect();crossed.sort_by(|a,b|b.1.cmp(&a.1).then(a.0.cmp(&b.0)));named=crossed.into_iter().map(|x|x.0).filter(|n|fresh(ctx,&next,n)).take(NAME_TOP).collect();}
    }
    for lane in &named {
        let mut failing:Vec<_>=hot_rows.iter().filter(|(n,_)|n==lane).map(|(_,e)|*e).collect();failing.sort();failing.dedup();
        if failing.is_empty(){failing=vec![0,1];}
        let widen=failing.len()==1&&plan_has_via_in_pad(if failing[0]==0{source_plan}else{destination_plan},ctx,lane);
        let original=failing[0];
        for end in failing {if let Some(item)=make_end(ctx,lane,end,round){put_avoid(&mut next,item,round,&mut raised);}}
        if widen {if let Some(item)=make_end(ctx,lane,1-original,round){put_avoid(&mut next,item,round,&mut raised);}}
    }
    if !named.is_empty(){let mut already:BTreeSet<_>=next.named.iter().cloned().collect();for lane in named{if already.insert(lane.clone()){next.named.push(lane);}}}
    next
}

fn fresh(ctx:&WholeContext,feedback:&EndsFeedback,lane:&str)->bool{
    (0..2).filter_map(|end|make_end(ctx,lane,end,0)).any(|item|!feedback.avoid.iter().any(|old|same_end(old,&item)))
}

/// Apply the pinned `--raise` transition: increment only endpoint prices last named in
/// this round; `widen` adds any missing endpoint of every previously named lane.
pub fn raise_round(ctx:&WholeContext,previous:&EndsFeedback,round:usize,widen:bool)->EndsFeedback{
    let mut next=previous.clone();next.points_in_flow_frame=true;
    for item in &mut next.avoid{if item.round==Some(round){item.times=item.times.max(1).saturating_add(1);}}
    if widen {
        for lane in &next.named {
            for end in 0..2 {
                if let Some(item)=make_end(ctx,lane,end,round){
                    if !next.avoid.iter().any(|old|same_end(old,&item)){next.avoid.push(item);}
                }
            }
        }
    }
    next
}

#[derive(Clone,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub struct HotAtEnd {pub kind:String,pub lanes:Vec<String>,pub end:usize}

/// Return the pinned `--repeat` intersection for two hot audit snapshots.
pub fn repeated_findings(ctx:&WholeContext,before:&HistoryAudit,now:&HistoryAudit)->Vec<HotAtEnd>{
    fn rows(ctx:&WholeContext,audit:&HistoryAudit)->BTreeSet<HotAtEnd>{
        let mut out=BTreeSet::new();
        for hot in &audit.hot {let mut lanes=lanes_named(ctx,&hot.lanes);lanes.sort();if lanes.is_empty(){continue;}
            let ends:BTreeSet<_>=at_end(ctx,hot).into_iter().map(|(_,end)|end).collect();
            for end in ends{out.insert(HotAtEnd{kind:hot.kind.clone(),lanes:lanes.clone(),end});}
        }
        out
    }
    rows(ctx,before).intersection(&rows(ctx,now)).cloned().collect()
}

/// Return only immediate endpoint findings that stop before another whole-route solve.
pub fn immediate_findings(ctx:&WholeContext,now:&HistoryAudit)->Vec<HotAtEnd>{
    let mut out=BTreeSet::new();
    for hot in &now.hot {if hot.kind!="PITCH"&&hot.kind!="STATIC"{continue;}let mut lanes=lanes_named(ctx,&hot.lanes);lanes.sort();
        let ends:BTreeSet<_>=at_end(ctx,hot).into_iter().map(|(_,end)|end).collect();
        for end in ends{out.insert(HotAtEnd{kind:hot.kind.clone(),lanes:lanes.clone(),end});}
    }
    out.into_iter().collect()
}

#[derive(Clone,Debug)]
pub struct RefusedSplit {pub pair_lane:String,pub end:usize,pub between:Vec<String>}

/// Apply the pinned fanout-audit `--refused` transition: split pairs become forbidden
/// end pairs, and far-face source teeth become endpoint avoids.
pub fn refused_round(ctx:&WholeContext,previous:&EndsFeedback,splits:&[RefusedSplit],far:&[String],round:usize)->EndsFeedback{
    let mut next=previous.clone();next.points_in_flow_frame=true;let mut raised=BTreeSet::new();
    for split in splits {if split.end>1{continue;}let a_lane=lane_for(ctx,&split.pair_lane);
        let Some(a)=make_end(ctx,&a_lane,split.end,round)else{continue;};
        for other in &split.between {let b_lane=lane_for(ctx,other);let Some(b)=make_end(ctx,&b_lane,split.end,round)else{continue;};
            let mut pair=[a.clone(),b];pair.sort_by(|x,y|x.lane.cmp(&y.lane));
            if !next.pairs.iter().any(|old|same_pair(old,&pair)){next.pairs.push(pair);}
        }
    }
    for lane in far {let lane=lane_for(ctx,lane);if let Some(item)=make_end(ctx,&lane,0,round){put_avoid(&mut next,item,round,&mut raised);}}
    next
}

/// Classify audit findings at the actual endpoint neighborhoods, in the board frame.
pub fn at_end(ctx:&WholeContext,hot:&super::whole_types::HotFinding)->Vec<(String,usize)>{
    let lanes=lanes_named(ctx,&hot.lanes);let mut out=Vec::new();
    for end in 0..2{if end_box(ctx,end).is_some_and(|b|near(ctx,hot.point,end,b)){out.extend(lanes.iter().cloned().map(|lane|(lane,end)));}}
    out.sort();out.dedup();out
}
