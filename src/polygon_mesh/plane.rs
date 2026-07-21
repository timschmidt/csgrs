//! Plane classification and clipping for retained planar polygon faces.

use crate::mesh::plane::{BACK, COPLANAR, FRONT, Plane, SPANNING, normals_same_direction};
use crate::vertex::Vertex;

use super::Polygon;

impl Plane {
    /// Classify a planar polygon relative to this plane.
    ///
    /// The returned bitmask combines `COPLANAR`, `FRONT`, and `BACK`.
    pub fn classify_polygon<M: Clone + Send + Sync>(&self, polygon: &Polygon<M>) -> i8 {
        polygon
            .vertices()
            .iter()
            .fold(COPLANAR, |classification, vertex| {
                classification | self.orient_point(&vertex.position)
            })
    }

    /// Split one retained planar polygon into coplanar/front/back buckets.
    #[allow(clippy::type_complexity)]
    pub fn split_polygon<M: Clone + Send + Sync>(
        &self,
        polygon: &Polygon<M>,
    ) -> (
        Vec<Polygon<M>>,
        Vec<Polygon<M>>,
        Vec<Polygon<M>>,
        Vec<Polygon<M>>,
    ) {
        let mut coplanar_front = Vec::new();
        let mut coplanar_back = Vec::new();
        let mut front = Vec::new();
        let mut back = Vec::new();
        let normal = self.normal();
        let unscaled_normal = self.unscaled_hreal_normal();
        let types = polygon
            .vertices()
            .iter()
            .map(|vertex| self.orient_point(&vertex.position))
            .collect::<Vec<_>>();
        let polygon_type = types.iter().fold(COPLANAR, |kind, next| kind | next);

        match polygon_type {
            COPLANAR => {
                if normals_same_direction(&normal, &polygon.plane().normal()) {
                    coplanar_front.push(polygon.clone());
                } else {
                    coplanar_back.push(polygon.clone());
                }
            },
            FRONT => front.push(polygon.clone()),
            BACK => back.push(polygon.clone()),
            _ => {
                let mut split_front = Vec::<Vertex>::new();
                let mut split_back = Vec::<Vertex>::new();
                for index in 0..polygon.vertices().len() {
                    let next = (index + 1) % polygon.vertices().len();
                    let type_current = types[index];
                    let type_next = types[next];
                    let current = &polygon.vertices()[index];
                    let next = &polygon.vertices()[next];

                    if type_current != BACK {
                        split_front.push(current.clone());
                    }
                    if type_current != FRONT {
                        split_back.push(current.clone());
                    }
                    if (type_current | type_next) == SPANNING
                        && let Some(parameter) = self.edge_intersection_parameter(
                            current,
                            next,
                            unscaled_normal.as_ref(),
                        )
                    {
                        let intersection = current.interpolate(next, parameter);
                        split_front.push(intersection.clone());
                        split_back.push(intersection);
                    }
                }
                push_split_polygon(&mut front, split_front, polygon);
                push_split_polygon(&mut back, split_back, polygon);
            },
        }
        (coplanar_front, coplanar_back, front, back)
    }
}

fn push_split_polygon<M: Clone + Send + Sync>(
    output: &mut Vec<Polygon<M>>,
    vertices: Vec<Vertex>,
    source: &Polygon<M>,
) {
    if vertices.len() >= 3 {
        output.push(
            Polygon::new(vertices, source.metadata().clone()).with_plane_id(source.plane_id),
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hyperlattice::{Point3, Real, Vector3};

    fn vertex(x: i32, y: i32) -> Vertex {
        Vertex::new(
            Point3::new(Real::from(x), Real::from(y), Real::zero()),
            Vector3::z(),
        )
    }

    #[test]
    fn split_polygon_stays_in_polygon_backend_and_retains_metadata() {
        let plane = Plane::from_normal(Vector3::y(), Real::zero());
        let polygon = Polygon::new(
            vec![vertex(-1, -1), vertex(1, -1), vertex(1, 1), vertex(-1, 1)],
            "split",
        );

        let (_, _, front, back) = plane.split_polygon(&polygon);

        assert_eq!(front.len(), 1);
        assert_eq!(back.len(), 1);
        assert_eq!(front[0].metadata(), &"split");
        assert_eq!(back[0].metadata(), &"split");
        assert!(
            front[0]
                .vertices()
                .iter()
                .all(|vertex| vertex.position.y >= Real::zero())
        );
        assert!(
            back[0]
                .vertices()
                .iter()
                .all(|vertex| vertex.position.y <= Real::zero())
        );
    }
}
