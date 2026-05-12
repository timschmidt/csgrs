use anyhow::Result;
use clap::Parser;

fn main() -> Result<()> {
    hyperdrc::app::run(hyperdrc::cli::Cli::parse())
}
