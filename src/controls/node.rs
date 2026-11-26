//! Subscribe to state and path, and publish commands. `run` is where you should implement your code, but `controlled_update` is already written.

use std::time::{Duration, Instant};

use prost::Message;
use zenoh::Wait;
use zenoh::handlers::RingChannel;

use crate::controls::Controller;
// Connect to the context
use crate::runtime::node::Context;
use crate::shared::protocol::pb::{StateUpdate};
use crate::shared::transport;
use crate::{
    shared::protocol::{self, pb},
    Result,
};

/// Still waiting for the state and path subscriptions, and the command publisher.
/// Copy the shape of `perception::node::run`.
pub fn run(mut ctx: Context) -> Result<()> {
    let states = ctx
        .transport
        .session
        .declare_subscriber(ctx.transport.key("state"))
        .with(RingChannel::new(32))
        .wait()?;
    
    let paths = ctx
        .transport
        .session
        .declare_subscriber(ctx.transport.key("path"))
        .with(RingChannel::new(1))
        .wait()?;

    let commands = ctx
        .transport
        .session
        .declare_publisher(ctx.transport.key("command"))
        .encoding(zenoh::bytes::Encoding::APPLICATION_PROTOBUF)
        .congestion_control(zenoh::qos::CongestionControl::Drop)
        .wait()?;

    ctx.status(
        true,
        "controls subscribed",
        0,
    )?;

    let mut last_instant = Instant::now();

    let start_time = Instant::now();

    let mut latest_time_ns = 0;
    let mut latest_sequence: Option<u64> = None;
    let mut latest_state: Option<StateUpdate> = None;
    let mut path: Option<pb::PathUpdate> = None;

    let mut controller = Controller::new(protocol::convert(&ctx.info.vehicle.as_ref().unwrap())?);

    while ctx.poll()? {
        while let Some(update) = transport::receive::<pb::StateUpdate>(&states, Duration::ZERO)? {
            let (h, _) = protocol::state(&update, &ctx.transport.run)?;
            if h.time_ns > latest_time_ns {
                latest_time_ns = h.time_ns;
                latest_sequence = Some(h.sequence);
                latest_state = Some(update);
            }
        }
        while let Some(update) = transport::receive::<pb::PathUpdate>(&paths, Duration::ZERO)? {
            path = Some(update);
            controller.index = 0;
        }

        let dt_s = Instant::now().duration_since(last_instant).as_secs_f64();
        if dt_s >= 0.1 {
            if let Some(state) = latest_state.as_ref() {
                if let Some(sequence) = latest_sequence {
                    commands.put(controlled_update(
                        &ctx,
                        state,
                        path.as_ref(),
                        &mut controller,
                        sequence,
                        dt_s
                    )?.encode_to_vec()).wait()?;
                    ctx.status(true, "command published", start_time.elapsed().as_nanos() as u64)?;
                    last_instant = Instant::now();
                }
            }
        }
    }

    Ok(())
    // // Edit me...
    // // Read StateUpdate and PathUpdate. Keep the latest path, even when it says there's nothing.
    // // Call controlled_update about 10 times a second and publish on "command".  //todo: rework to fit this
    // ctx.wait_for_wiring(
    //     "controls still needs its Zenoh subscribers and publisher; see docs/project.md, Connecting the pieces",
    // )
    // // ...till here.
}

/// Build a command from the latest state and path.
/// Missing, unready, or expired paths, and a bad `dt_s`, become a stop.
/// Publishing again does not refresh an old path's deadline. The simulator clamps the command later.
pub fn controlled_update(
    ctx: &Context,
    state: &pb::StateUpdate,
    path: Option<&pb::PathUpdate>,
    controller: &mut crate::controls::Controller,
    sequence: u64,
    dt_s: f64,
) -> Result<pb::CommandUpdate> {
    let (h, state) = protocol::state(state, &ctx.transport.run)?;
    let mut out = pb::CommandUpdate {
        header: Some(protocol::header(
            &ctx.transport.run,
            sequence,
            h.time_ns,
            "base_link",
        )),
        state_sequence: h.sequence,
        control_lease_id: ctx.info.control_lease_id.clone(),
        command: Some(protocol::convert(
            &crate::shared::types::DriveCommand::STOP,
        )?),
        ..Default::default()
    };
    let Some(path) = path else {
        println!("bruh 1");
        return Ok(out);
    };
    if path.availability != pb::Availability::Ready as i32 {
        println!("{}", path.availability);
        return Ok(out);
    }
    let path_h = protocol::check_header(&path.header, &ctx.transport.run, "map")?;
    let source = protocol::check_header(&path.source, &ctx.transport.run, "map")?;
    if !dt_s.is_finite()
        || dt_s <= 0.
        || h.time_ns
            .checked_sub(source.time_ns)
            .is_none_or(|age| age >= protocol::FRESH_NS)
    {
        println!("bruh 3");
        return Ok(out);
    }
    let points: Vec<crate::shared::types::Point2> = protocol::convert(
        &path
            .path
            .as_ref()
            .ok_or_else(|| crate::error("Missing ready path"))?
            .points,
    )?;
    if points.is_empty() || points.len() > 100_000 || points.iter().any(|p| !p.is_finite()) {
        return Err(crate::error("Invalid path points"));
    }
    let command = controller.command(&state, &points, dt_s);
    // The number we publish is the one the controller asked for. Clamping happens later.
    controller.vehicle.require_finite(command)?;
    out.command = Some(protocol::convert(&command)?);
    out.source = Some(source.clone());
    out.path_sequence = path_h.sequence;
    out.valid_until_ns = source.time_ns + protocol::FRESH_NS;
    Ok(out)
}
