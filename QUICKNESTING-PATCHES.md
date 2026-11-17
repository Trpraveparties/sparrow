# Patched libraries

`jagua-rs` and `sparrow` are by Jeroen Gardeyn and are licensed under the
Mozilla Public License 2.0 (each directory keeps its `LICENSE`). Quick
Nesting ships them inside its WebAssembly engine, so under MPL-2.0 the
source of these files, with our changes, is published:

- jagua-rs: https://github.com/Trpraveparties/jagua-rs, branch `quicknesting-outline`
  (upstream https://github.com/JeroenGar/jagua-rs, revision `877aaa3c3217f0dd2b6f1c8af8d3e53d4023e422`)
- sparrow: https://github.com/Trpraveparties/sparrow, branch `quicknesting-outline`
  (upstream https://github.com/JeroenGar/sparrow, revision `04f54ff77fd9b614311879e8c62ee4e13294165e`)

The copies here are those revisions with the changes below and nothing else.
The workspace `Cargo.toml` points both git dependencies at these copies with
`[patch]`.

## What changed, and why

Both changes let a sheet be any outline (an offcut) instead of a rectangle.
A rectangular sheet takes exactly the code path it took before.

### jagua-rs

- `jagua-rs/src/probs/spp/entities/strip.rs`: `Strip` gains
  `outline: Option<&'static [Point]>` and `Strip::with_outline`. When an
  outline is set, the strip's container is that polygon whatever the width
  says. `Strip::new` leaves it `None`, so existing callers are unchanged.

### sparrow

- `src/quantify/mod.rs`: new `quantify_collision_poly_container_shape`. For a
  rectangular container it calls the original
  `quantify_collision_poly_container` with the bounding box. For any other
  outline it measures how far the item sticks out of the outline: each pole
  of the item's surrogate by its depth past the outline, each vertex outside
  by its distance, plus the same small floor.
- `src/quantify/tracker.rs`, `src/eval/specialized_jaguars_pipeline.rs`,
  `src/util/assertions.rs`: call the new function with the container's shape
  instead of its bounding box.

The vendored copies leave out upstream's tests, benchmarks, data and images.
