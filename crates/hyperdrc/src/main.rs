use anyhow::Result;
use clap::Parser;

fn main() -> Result<()> {
    hyperdrc::run_cli(hyperdrc::Cli::parse())
}
