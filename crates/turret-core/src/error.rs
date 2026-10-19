//! Typed failures at the portable payload boundary.

/// A rejected command or invalid state transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum Error {
    /// A quantity is nonfinite or outside its unit's domain.
    #[error("invalid numeric value")]
    InvalidValue,
    /// A capability contains inconsistent limits.
    #[error("invalid capability limits")]
    InvalidLimits,
    /// The device does not implement the operation.
    #[error("operation is unsupported")]
    Unsupported,
    /// Support has not been established.
    #[error("operation support is unknown")]
    UnknownCapability,
    /// A supported operation cannot currently be used.
    #[error("operation is unavailable")]
    Unavailable,
    /// A requested quantity exceeds the advertised bound.
    #[error("requested value is outside capability limits")]
    OutOfRange,
    /// The command references an unadvertised coordinate frame.
    #[error("coordinate frame is unsupported")]
    UnsupportedFrame,
    /// A partial command does not contain any axis.
    #[error("command has no selected axis")]
    EmptyCommand,
    /// A command belongs to a different connection or authority generation.
    #[error("command identity is obsolete")]
    StaleIdentity,
    /// A command was based on different capability information.
    #[error("capability revision has changed")]
    CapabilityRevision,
    /// A sequence is duplicated, older, or ambiguously distant.
    #[error("command sequence is not newer")]
    StaleSequence,
    /// The executor clock moved backwards.
    #[error("executor monotonic clock moved backwards")]
    ClockRegression,
    /// The command's deadlines are inconsistent.
    #[error("invalid command deadlines")]
    InvalidTiming,
    /// A continuous command does not carry an execution expiry.
    #[error("continuous commands require a validity deadline")]
    MissingValidity,
    /// The command expired before a write could start.
    #[error("command expired before transmission")]
    Expired,
    /// The requested command is not in the relevant executor slot.
    #[error("command is not pending or active")]
    NotFound,
    /// A write was reported without passing the immediate write-boundary check.
    #[error("command was not prepared for transmission")]
    NotPrepared,
    /// The operation requires a command that has actually been transmitted.
    #[error("command has not been transmitted")]
    NotSent,
    /// A terminal command cannot advance to another lifecycle stage.
    #[error("command already ended")]
    Ended,
    /// An observation cannot establish physical completion.
    #[error("observation is not a fresh device measurement")]
    InvalidObservation,
    /// The command is not an absolute target with observable completion.
    #[error("command has no observable absolute target")]
    NoCompletionTarget,
}
