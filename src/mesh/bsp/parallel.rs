//! Parallel implementation of BSP operations

#[cfg(feature = "parallel")]
use rayon::prelude::*;

use crate::float_types::EPSILON;
use crate::mesh::bsp::node::Node;
use crate::mesh::bsp::traits::{BalancedSplittingStrategy, BspOps, SplittingPlaneStrategy};
use crate::mesh::plane::{BACK, COPLANAR, FRONT, Plane, SPANNING};
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use std::fmt::Debug;

/// Parallel implementation of BSP operations
#[cfg(feature = "parallel")]
pub struct ParallelBspOps<
    SP: SplittingPlaneStrategy<S> = BalancedSplittingStrategy,
    S: Clone = (),
> {
    splitting_strategy: SP,
    _phantom: std::marker::PhantomData<S>,
}

#[cfg(feature = "parallel")]
impl<S: Clone> ParallelBspOps<BalancedSplittingStrategy, S> {
    pub fn new() -> Self {
        Self {
            splitting_strategy: BalancedSplittingStrategy::default(),
            _phantom: std::marker::PhantomData,
        }
    }
}

#[cfg(feature = "parallel")]
impl<S: Clone> Default for ParallelBspOps<BalancedSplittingStrategy, S> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "parallel")]
impl<SP: SplittingPlaneStrategy<S>, S: Clone> ParallelBspOps<SP, S> {
    pub const fn with_strategy(strategy: SP) -> Self {
        Self {
            splitting_strategy: strategy,
            _phantom: std::marker::PhantomData,
        }
    }
}

#[cfg(feature = "parallel")]
impl<SP: SplittingPlaneStrategy<S> + Sync, S: Clone + Send + Sync + Debug> BspOps<S>
    for ParallelBspOps<SP, S>
{
    fn invert(&self, node: &mut Node<S>) {
        // Use iterative approach with a stack to avoid stack overflow
        let mut stack = vec![node];

        while let Some(current) = stack.pop() {
            // Flip all polygons and plane in this node
            current.polygons.par_iter_mut().for_each(|p| p.flip());
            if let Some(ref mut plane) = current.plane {
                plane.flip();
            }

            // Swap front and back children
            std::mem::swap(&mut current.front, &mut current.back);

            // Add children to stack for processing
            if let Some(ref mut front) = current.front {
                stack.push(front.as_mut());
            }
            if let Some(ref mut back) = current.back {
                stack.push(back.as_mut());
            }
        }
    }

    fn clip_polygons(&self, node: &Node<S>, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        // If this node has no plane, just return the original set
        if node.plane.is_none() {
            return polygons.to_vec();
        }
        let plane = node.plane.as_ref().unwrap();

        // Split each polygon in parallel; gather results
        let (coplanar_front, coplanar_back, mut front, mut back) = polygons
            .par_iter()
            .map(|poly| plane.split_polygon(poly))
            .reduce(
                || (Vec::new(), Vec::new(), Vec::new(), Vec::new()),
                |mut acc, x| {
                    acc.0.extend(x.0);
                    acc.1.extend(x.1);
                    acc.2.extend(x.2);
                    acc.3.extend(x.3);
                    acc
                },
            );

        // Decide where to send the coplanar polygons
        coplanar_front
            .into_iter()
            .chain(coplanar_back)
            .for_each(|cp| {
                if plane.orient_plane(&cp.plane) == FRONT {
                    front.push(cp);
                } else {
                    back.push(cp);
                }
            });

        // Process front and back using parallel iterators to avoid recursive join
        let mut result = if let Some(ref f) = node.front {
            self.clip_polygons(f, &front)
        } else {
            front
        };

        if let Some(ref b) = node.back {
            result.extend(self.clip_polygons(b, &back));
        }

        result
    }

    fn clip_to(&self, node: &mut Node<S>, bsp: &Node<S>) {
        // Use iterative approach with a stack to avoid recursive stack overflow
        let mut stack = vec![node];

        while let Some(current) = stack.pop() {
            // Clip polygons at this node
            current.polygons = self.clip_polygons(bsp, &current.polygons);

            // Add children to stack for processing
            if let Some(ref mut front) = current.front {
                stack.push(front.as_mut());
            }
            if let Some(ref mut back) = current.back {
                stack.push(back.as_mut());
            }
        }
    }

    fn build(&self, node: &mut Node<S>, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }

        // Choose splitting plane if not already set
        if node.plane.is_none() {
            node.plane = Some(self.splitting_strategy.pick_best_splitting_plane(polygons));
        }
        let plane = node.plane.as_ref().unwrap();

        // Split polygons in parallel
        let (mut coplanar_front, mut coplanar_back, front, back) =
            polygons.par_iter().map(|p| plane.split_polygon(p)).reduce(
                || (Vec::new(), Vec::new(), Vec::new(), Vec::new()),
                |mut acc, x| {
                    acc.0.extend(x.0);
                    acc.1.extend(x.1);
                    acc.2.extend(x.2);
                    acc.3.extend(x.3);
                    acc
                },
            );

        // Append coplanar fronts/backs to node.polygons
        node.polygons.append(&mut coplanar_front);
        node.polygons.append(&mut coplanar_back);

        // Build children sequentially to avoid stack overflow from recursive join
        if !front.is_empty() {
            let mut front_node = node.front.take().unwrap_or_else(|| Box::new(Node::new()));
            self.build(&mut front_node, &front);
            node.front = Some(front_node);
        }

        if !back.is_empty() {
            let mut back_node = node.back.take().unwrap_or_else(|| Box::new(Node::new()));
            self.build(&mut back_node, &back);
            node.back = Some(back_node);
        }
    }

    fn all_polygons(&self, node: &Node<S>) -> Vec<Polygon<S>> {
        // Use serial version as parallel collection doesn't provide significant benefit here
        let mut result = Vec::new();
        let mut stack = vec![node];

        while let Some(current) = stack.pop() {
            result.extend_from_slice(&current.polygons);

            // Use iterator to add child nodes more efficiently
            stack.extend(
                [&current.front, &current.back]
                    .iter()
                    .filter_map(|child| child.as_ref().map(|boxed| boxed.as_ref())),
            );
        }
        result
    }

    fn slice(
        &self,
        node: &Node<S>,
        slicing_plane: &Plane,
    ) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        // Collect all polygons
        let all_polys = self.all_polygons(node);

        // Process polygons in parallel
        let (coplanar_polygons, intersection_edges) = all_polys
            .par_iter()
            .map(|poly| {
                let vcount = poly.vertices.len();
                if vcount < 2 {
                    // Degenerate => skip
                    return (Vec::new(), Vec::new());
                }

                let types: Vec<_> = poly
                    .vertices
                    .iter()
                    .map(|vertex| slicing_plane.orient_point(&vertex.pos))
                    .collect();

                let polygon_type = types.iter().fold(0, |acc, &vertex_type| acc | vertex_type);

                match polygon_type {
                    COPLANAR => {
                        // Entire polygon in plane
                        (vec![poly.clone()], Vec::new())
                    },
                    FRONT | BACK => {
                        // Entirely on one side => no intersection
                        (Vec::new(), Vec::new())
                    },
                    SPANNING => {
                        // The polygon crosses the plane => gather intersection edges
                        let crossing_points: Vec<_> = (0..vcount)
                            .filter_map(|i| {
                                let j = (i + 1) % vcount;
                                let ti = types[i];
                                let tj = types[j];
                                let vi = &poly.vertices[i];
                                let vj = &poly.vertices[j];

                                if (ti | tj) == SPANNING {
                                    // The param intersection at which plane intersects the edge
                                    let denom = slicing_plane.normal().dot(&(vj.pos - vi.pos));
                                    if denom.abs() > EPSILON {
                                        let intersection = (slicing_plane.offset()
                                            - slicing_plane.normal().dot(&vi.pos.coords))
                                            / denom;
                                        // Interpolate:
                                        Some(vi.interpolate(vj, intersection))
                                    } else {
                                        None
                                    }
                                } else {
                                    None
                                }
                            })
                            .collect();

                        // Pair up intersection points => edges
                        let edges: Vec<_> = crossing_points
                            .chunks_exact(2)
                            .map(|chunk| [chunk[0].clone(), chunk[1].clone()])
                            .collect();

                        (Vec::new(), edges)
                    },
                    _ => (Vec::new(), Vec::new()),
                }
            })
            .reduce(
                || (Vec::new(), Vec::new()),
                |mut acc, x| {
                    acc.0.extend(x.0);
                    acc.1.extend(x.1);
                    acc
                },
            );

        (coplanar_polygons, intersection_edges)
    }
}
