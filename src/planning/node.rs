//! Subscribe to maps and publish paths. `run` is yours. `planned_update` is already written.
use std::time::Duration;

use zenoh::Wait;
use zenoh::handlers::RingChannel;

use prost::Message;
use crate::planning::Planner;
use crate::runtime::node::Context;
use crate::shared::transport;
use crate::{
    shared::protocol::{self, pb},
    Result,
};

/// Still waiting for the map subscription and the path publisher.
/// Copy the shape of `perception::node::run`.
pub fn run(mut ctx: Context) -> Result<()> {
    let maps = ctx
        .transport
        .session
        .declare_subscriber(ctx.transport.key("map"))
        .with(RingChannel::new(1))
        .wait()?;

    let path = ctx
        .transport
        .session
        .declare_publisher(ctx.transport.key("path"))
        .encoding(zenoh::bytes::Encoding::APPLICATION_PROTOBUF)
        .congestion_control(zenoh::qos::CongestionControl::Drop)
        .wait()?;
    
    ctx.status(
        true,
        "planner subscribed",
        0,
    )?;

    // let mut map_fresh = Freshness::default();
    let mut planner = Planner::default();

    while ctx.poll()? {
        while let Some(map) = transport::receive::<pb::MapUpdate>(&maps, Duration::ZERO)? {
            // map_fresh.accept(&map.header);
            let path_update = planned_update(&ctx, &map, &mut planner)?;
            path.put(path_update.encode_to_vec()).wait()?;
            ctx.status(
                true,
                "yay",
                match map.header {
                    Some(h) => h.time_ns,
                    None => 0
                }
            )?;
        }
    }

    Result::Ok(())
}

/// Turn a map into a path message.
/// A map that isn't ready stays that way. `None` from the planner becomes "no path."
/// A path that fails the clearance check is an error.
/// The map's source time is copied onto the path, so an old plan doesn't look new.
pub fn planned_update(
    ctx: &Context,
    map: &pb::MapUpdate,
    planner: &mut crate::planning::Planner,
) -> Result<pb::PathUpdate> {
    let h = protocol::check_header(&map.header, &ctx.transport.run, "map")?;
    let source = protocol::check_header(&map.source, &ctx.transport.run, "map")?;
    let mut out = pb::PathUpdate {
        header: Some(h.clone()),
        source: Some(source),
        map_sequence: h.sequence,
        path: None,
        availability: pb::Availability::NotReady as i32,
    };
    if map.availability != pb::Availability::Ready as i32 {
        return Ok(out);
    }
    let grid: crate::shared::grid::Grid = protocol::convert(
        map.grid
            .as_ref()
            .ok_or_else(|| crate::error("Missing ready map"))?,
    )?;
    grid.validate()?;
    let state: crate::shared::types::VehicleState = protocol::convert(
        map.state
            .as_ref()
            .ok_or_else(|| crate::error("Missing capture state"))?,
    )?;
    let goal = protocol::convert(
        ctx.info
            .goal
            .as_ref()
            .ok_or_else(|| crate::error("Missing goal"))?,
    )?;
    let spec: crate::shared::types::VehicleSpec = protocol::convert(
        ctx.info
            .vehicle
            .as_ref()
            .ok_or_else(|| crate::error("Missing vehicle"))?,
    )?;
    match planner.plan(
        &grid,
        &state,
        goal,
        spec.clearance_m(),
        ctx.info.goal_tolerance_m,
    ) {
        Some(path) => {
            crate::planning::validation::validate_path(
                &grid,
                &path,
                state.pose.position,
                goal,
                spec.clearance_m(),
                ctx.info.goal_tolerance_m,
            )?;
            out.path = Some(pb::Path {
                points: protocol::convert(&path)?,
            });
            out.availability = pb::Availability::Ready as i32;
        }
        None => out.availability = pb::Availability::NoPath as i32,
    }
    Ok(out)
}
