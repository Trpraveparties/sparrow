use jagua_rs::geometry::geo_traits::DistanceTo;
use jagua_rs::geometry::primitives::{Rect, SPolygon};
use crate::consts::OVERLAP_PROXY_EPSILON_DIAM_RATIO;
use crate::quantify::overlap_proxy::overlap_area_proxy;

pub mod overlap_proxy;
mod pair_matrix;
pub mod tracker;
#[cfg(feature = "simd")]
pub mod simd;

/// Quantifies a collision between two simple polygons.
/// Algorithm 4 from https://doi.org/10.48550/arXiv.2509.13329
#[inline(always)]
pub fn quantify_collision_poly_poly(s1: &SPolygon, s2: &SPolygon) -> f32 {
    let epsilon = f32::max(s1.diameter, s2.diameter) * OVERLAP_PROXY_EPSILON_DIAM_RATIO;

    let overlap_proxy = overlap_area_proxy(&s1.surrogate(), &s2.surrogate(), epsilon) + epsilon.powi(2);

    debug_assert!(overlap_proxy.is_normal());

    let penalty = calc_shape_penalty(s1, s2);

    overlap_proxy.sqrt() * penalty
}

pub fn calc_shape_penalty(s1: &SPolygon, s2: &SPolygon) -> f32 {
    let p1 = f32::sqrt(s1.surrogate().convex_hull_area);
    let p2 = f32::sqrt(s2.surrogate().convex_hull_area);
    (p1 * p2).sqrt() //geometric mean
}

/// Quantifies a collision between a simple polygon and the exterior of the
/// container, whatever the container's shape.
///
/// Patched for Quick Nesting (see vendor/PATCHES.md in nest-engine): upstream
/// only ever has rectangular containers and measures against the bounding
/// box. A rectangle still takes exactly that path; any other outline (an
/// offcut sheet) is measured against the outline itself.
#[inline(always)]
pub fn quantify_collision_poly_container_shape(s: &SPolygon, c: &SPolygon) -> f32 {
    if is_rectangle(c) {
        return quantify_collision_poly_container(s, c.bbox);
    }
    quantify_collision_poly_outline(s, c)
}

/// True when `c` is its own bounding box (four corners, same area).
#[inline(always)]
fn is_rectangle(c: &SPolygon) -> bool {
    c.vertices.len() == 4 && (c.bbox.area() - c.area).abs() <= 1e-4 * c.bbox.area()
}

/// How far `s` sticks out of the outline `c`. Each of `s`'s poles (the
/// circles that stand in for its body) counts by how deep it crosses the
/// outline, and each vertex outside by its distance from it, so both a body
/// hanging over the edge and a thin spike through it register and shrink as
/// the part moves back in. A small floor keeps the loss positive for any
/// colliding pair, as the rectangle version does.
fn quantify_collision_poly_outline(s: &SPolygon, c: &SPolygon) -> f32 {
    use jagua_rs::geometry::geo_traits::CollidesWith;
    let s_bbox = s.bbox;
    if Rect::intersection(s_bbox, c.bbox).is_none() {
        let overlap = s_bbox.area() + s_bbox.centroid().distance_to(&c.bbox.centroid());
        return 2.0 * overlap.sqrt() * calc_shape_penalty(s, s);
    }
    let boundary_distance = |p: &jagua_rs::geometry::primitives::Point| -> f32 {
        c.edge_iter()
            .map(|e| e.sq_distance_to(p))
            .fold(f32::INFINITY, f32::min)
            .sqrt()
    };
    let mut outside = 0.0001 * s_bbox.area();
    for pole in &s.surrogate().poles {
        let d = boundary_distance(&pole.center);
        // signed: positive inside the outline
        let signed = if c.collides_with(&pole.center) { d } else { -d };
        let depth = (pole.radius - signed).clamp(0.0, 2.0 * pole.radius);
        outside += 2.0 * depth * pole.radius;
    }
    for v in &s.vertices {
        if !c.collides_with(v) {
            let d = boundary_distance(v);
            outside += d * d;
        }
    }
    debug_assert!(outside.is_normal());
    2.0 * outside.sqrt() * calc_shape_penalty(s, s)
}

/// Quantifies a collision between a simple polygon and the exterior of the container.
#[inline(always)]
pub fn quantify_collision_poly_container(s: &SPolygon, c_bbox: Rect) -> f32 {
    let s_bbox = s.bbox;
    let overlap = match Rect::intersection(s_bbox, c_bbox) {
        Some(r) => {
            //intersection exist, calculate the area of the intersection (+ a small value to ensure it is never zero)
            let negative_area = (s_bbox.area() - r.area()) + 0.0001 * s_bbox.area();
            negative_area
        }
        None => {
            //no intersection, guide towards intersection with container
            s_bbox.area() + s_bbox.centroid().distance_to(&c_bbox.centroid())
        }
    };
    debug_assert!(overlap.is_normal());

    let penalty = calc_shape_penalty(s, s);

    2.0 * overlap.sqrt() * penalty
}