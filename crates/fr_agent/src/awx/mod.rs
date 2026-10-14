//! Complete AWX port in progress; unfinished stages reject execution explicitly.
pub mod conflict_groups;
pub mod corridor;
pub mod cp_model;
pub mod cp_sat_bridge;
pub mod escape_moves;
pub mod fab_rules;
pub mod joint_escape;
pub mod joint_geometry;
pub mod numeric;
pub mod pairs;
pub mod source_realize;
pub mod whole_audit;
pub mod whole_context;
pub mod whole_frame;
pub mod whole_solve;
pub mod whole_types;

pub mod feedback;

pub mod linear_program;

pub mod whole_geometry;
pub mod whole_polish;

pub mod board_snapshot;

pub mod whole_snap;

pub mod native_execution;

pub mod whole_route;
pub mod relayer;

pub mod ends_geometry;
pub mod whole_ends;
pub mod whole_feedback;
pub mod taut_paths;
pub mod context_builder;
pub mod integration;
pub mod passive_clearance;
pub mod fanout_driver;
pub mod runtime_stats;
pub mod bus_discovery;
pub mod route_ladder;
pub mod route_recovery;
pub mod destination_realize;
pub mod lane_order;
pub mod smoothing_chains;
pub mod board_smoothing;
pub mod route_evolution;
pub mod evolution_native;
pub mod evolution_descent;
pub mod evolution_probes;
pub mod replan_state;
pub mod replan_types;
pub mod replan_probes;
pub mod replan_finalize;
pub mod native_replan;
pub mod replan_cost;
pub mod replan_buses;
pub mod braid_schedule;
pub mod replan_prediction;
pub mod braid_plan;
pub mod braid_corridor;
pub mod braid_crossings;
pub mod braid_setup;
pub mod braid_geometry;
pub mod taut_spine;
pub mod braid_offsets;
pub mod braid_lanes;
pub mod evolution_export;
pub mod braid_connect;
pub mod braid_band;
pub mod native_rung;
pub mod pair_connection;
pub mod pair_geometry;
pub mod native_obstacles;
pub mod native_bga_zones;
pub mod grid_path;
pub mod native_grid_window;
pub mod native_net_ties;
pub mod native_outline;
pub mod native_via_obstacles;
pub mod native_same_net;
pub mod native_pair_router;
pub mod pair_validation;
pub mod pair_path;
pub mod via_pad_tap;
