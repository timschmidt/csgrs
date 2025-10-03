# CSGRS WASM Nuxt4 starter template

Based on official nuxt4 starter template. 

## Setup

## CSGRS WASM

Compile the wasm and JS directly from the main csgrs repo.

```bash
# make sure you have Rust and Cargo: https://doc.rust-lang.org/cargo/getting-started/installation.html
cargo install wasm-pack
wasm-pack build --release --target bundler --out-dir pkg -- --features wasm
```

## Nuxt4 setup

Make sure to install dependencies:

```bash
npm install
# or
pnpm install
# or
yarn install
# or
bun install
```

## Development Server

Start the development server on `http://localhost:3000`:

```bash
npm run dev
pnpm dev
yarn dev
bun run dev
```

## Production

Build the application for production:

```bash
npm run build
pnpm build
yarn build
bun run build
```

Locally preview production build:

```bash
npm run preview
pnpm preview
yarn preview
bun run preview
```

Check out the [deployment documentation](https://nuxt.com/docs/getting-started/deployment) for more information.
