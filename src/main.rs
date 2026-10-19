//! Standalone Turret runtime.

mod cli;

fn main() -> turret::Result<()> {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?
        .block_on(cli::run())
}
