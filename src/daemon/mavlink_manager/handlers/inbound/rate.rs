//! Preserve rate semantics until the actual device backend selects native or synthesized control.

use super::common::{dispatch_gimbal_command, rate_flags_supported};
use crate::daemon::mavlink_manager::RxCtx;
use crate::daemon::models::{CommandOperation, ControlSource, GimbalCommand};

/// Finite zero is a stop demand; NaN is an omitted axis.
pub(super) fn any_rate_active(rates: (f32, f32, f32)) -> bool {
    [rates.0, rates.1, rates.2]
        .iter()
        .any(|rate| rate.is_finite())
}

/// Send selected rates without inventing a position baseline at ingress.
pub(super) async fn dispatch_rate_command(
    source: ControlSource,
    rates: (f32, f32, f32),
    flags: mavlink::ardupilotmega::GimbalManagerFlags,
    ctx: &RxCtx<'_>,
    label: &'static str,
) -> bool {
    if !rate_flags_supported(flags) {
        return false;
    }
    let selected = |rate: f32| (!rate.is_nan()).then_some(rate);
    let mut command = GimbalCommand::new(
        source,
        selected(rates.0),
        selected(rates.1),
        selected(rates.2),
    );
    command.operation = CommandOperation::Rate;
    dispatch_gimbal_command(command, ctx, label).await
}
