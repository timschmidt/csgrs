//! Serial implementation of BSP operations

use crate::float_types::EPSILON;
use crate::mesh::bsp::node::Node;
use crate::mesh::bsp::traits::{BalancedSplittingStrategy, BspOps, SplittingPlaneStrategy};
use crate::mesh::plane::{Plane, BACK, COPLANAR, FRONT, SPANNING};
use crate::mesh::polygon::Polygon;
use crate::mesh::vertex::Vertex;
use std::fmt::Debug;

/// Serial implementation of BSP operations
pub struct SerialBspOps<SP: SplittingPlaneStrategy<S> = BalancedSplittingStrategy, S: Clone = ()> {
    splitting_strategy: SP,
    _phantom: std::marker::PhantomData<S>,
}

impl<S: Clone> SerialBspOps<BalancedSplittingStrategy, S> {
    pub fn new() -> Self {
        Self {
            splitting_strategy: BalancedSplittingStrategy::default(),
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<SP: SplittingPlaneStrategy<S>, S: Clone> SerialBspOps<SP, S> {
    pub fn with_strategy(strategy: SP) -> Self {
        Self {
            splitting_strategy: strategy,
            _phantom: std::marker::PhantomData,
        }
    }
}



impl<SP: SplittingPlaneStrategy<S>, S: Clone + Send + Sync + Debug> BspOps<S> for SerialBspOps<SP, S> {
    fn invert(&self, node: &mut Node<S>) {
        // Use iterative approach with a stack
        let mut stack = vec![node];

        while let Some(current) = stack.pop() {
            // Flip all polygons and plane in this node
            current.polygons.iter_mut().for_each(|p| p.flip());
            if let Some(ref mut plane) = current.plane {
                plane.flip();
            }

            // Swap front and back
            std::mem::swap(&mut current.front, &mut current.back);

            // Add children to stack
            if let Some(ref mut front) = current.front {
                stack.push(front.as_mut());
            }
            if let Some(ref mut back) = current.back {
                stack.push(back.as_mut());
            }
        }
    }

    fn clip_polygons(&self, node: &Node<S>, polygons: &[Polygon<S>]) -> Vec<Polygon<S>> {
        // If this node has no plane, just return
        if node.plane.is_none() {
            return polygons.to_vec();
        }

        let plane = node.plane.as_ref().unwrap();

        // Pre-allocate for better performance
        let mut front_polys = Vec::with_capacity(polygons.len());
        let mut back_polys = Vec::with_capacity(polygons.len());

        // Optimized polygon splitting with iterator patterns
        for polygon in polygons {
            let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) =
                plane.split_polygon(polygon);

            // Efficient coplanar polygon classification using iterator chain
            coplanar_front
                .into_iter()
                .chain(coplanar_back.into_iter())
                .for_each(|coplanar_poly| {
                    if plane.orient_plane(&coplanar_poly.plane) == FRONT {
                        front_parts.push(coplanar_poly);
                    } else {
                        back_parts.push(coplanar_poly);
                    }
                });

            front_polys.append(&mut front_parts);
            back_polys.append(&mut back_parts);
        }

        // Recursively clip with optimized pattern
        let mut result = if let Some(front_node) = &node.front {
            self.clip_polygons(front_node, &front_polys)
        } else {
            front_polys
        };

        if let Some(back_node) = &node.back {
            result.extend(self.clip_polygons(back_node, &back_polys));
        }

        result
    }

    fn clip_to(&self, node: &mut Node<S>, bsp: &Node<S>) {
        node.polygons = self.clip_polygons(bsp, &node.polygons);
        
        if let Some(ref mut front) = node.front {
            self.clip_to(front, bsp);
        }
        
        if let Some(ref mut back) = node.back {
            self.clip_to(back, bsp);
        }
    }

    fn all_polygons(&self, node: &Node<S>) -> Vec<Polygon<S>> {
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

    fn build(&self, node: &mut Node<S>, polygons: &[Polygon<S>]) {
        if polygons.is_empty() {
            return;
        }

        // Choose the best splitting plane if not already set
        if node.plane.is_none() {
            node.plane = Some(self.splitting_strategy.pick_best_splitting_plane(polygons));
        }
        let plane = node.plane.as_ref().unwrap();

        // Pre-allocate with estimated capacity
        let mut front = Vec::with_capacity(polygons.len() / 2);
        let mut back = Vec::with_capacity(polygons.len() / 2);

        // Optimized polygon classification using iterator pattern
        for polygon in polygons {
            let (coplanar_front, coplanar_back, mut front_parts, mut back_parts) =
                plane.split_polygon(polygon);

            // Extend collections efficiently with iterator chains
            node.polygons.extend(coplanar_front);
            node.polygons.extend(coplanar_back);
            front.append(&mut front_parts);
            back.append(&mut back_parts);
        }

        // Build child nodes using lazy initialization pattern
        if !front.is_empty() {
            node.front
                .get_or_insert_with(|| Box::new(Node::new()));
            self.build(node.front.as_mut().unwrap(), &front);
        }

        if !back.is_empty() {
            node.back
                .get_or_insert_with(|| Box::new(Node::new()));
            self.build(node.back.as_mut().unwrap(), &back);
        }
    }

    fn slice(&self, node: &Node<S>, slicing_plane: &Plane) -> (Vec<Polygon<S>>, Vec<[Vertex; 2]>) {
        let all_polys = self.all_polygons(node);

        let mut coplanar_polygons = Vec::new();
        let mut intersection_edges = Vec::new();

        for poly in &all_polys {
            let vcount = poly.vertices.len();
            if vcount < 2 {
                continue; // degenerate polygon => skip
            }

            // Use iterator to compute vertex types
            let types: Vec<_> = poly
                .vertices
                .iter()
                .map(|vertex| slicing_plane.orient_point(&vertex.pos))
                .collect();

            let polygon_type = types.iter().fold(0, |acc, &vertex_type| acc | vertex_type);

            match polygon_type {
                COPLANAR => {
                    coplanar_polygons.push(poly.clone());
                },
                FRONT | BACK => {
                    // Entirely on one side => no intersection
                },
                SPANNING => {
                    // The polygon crosses the plane
                    let crossing_points: Vec<_> = (0..vcount)
                        .filter_map(|i| {
                            let j = (i + 1) % vcount;
                            let ti = types[i];
                            let tj = types[j];
                            let vi = &poly.vertices[i];
                            let vj = &poly.vertices[j];

                            if (ti | tj) == SPANNING {
                                let denom = slicing_plane.normal().dot(&(vj.pos - vi.pos));
                                if denom.abs() > EPSILON {
                                    let intersection = (slicing_plane.offset()
                                        - slicing_plane.normal().dot(&vi.pos.coords))
                                        / denom;
                                    Some(vi.interpolate(vj, intersection))
                                } else {
                                    None
                                }
                            } else {
                                None
                            }
                        })
                        .collect();

                    // Convert crossing points to intersection edges
                    intersection_edges.extend(
                        crossing_points
                            .chunks_exact(2)
                            .map(|chunk| [chunk[0].clone(), chunk[1].clone()]),
                    );
                },
                _ => {
                    // Shouldn't happen in typical classification
                },
            }
        }

        (coplanar_polygons, intersection_edges)
    }
} 