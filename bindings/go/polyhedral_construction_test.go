package csgrs

import (
	"errors"
	"strings"
	"testing"
)

func TestPolyhedronRejectsCrossedFaceAndRecovers(t *testing.T) {
	mesh, err := TriangleMeshF64Polyhedron([]Vec3F64{{0, 0, 0}, {3, 3, 0}, {0, 2, 0}, {2, 0, 0}}, []uint{0, 1, 2, 3}, []uint{0, 4})
	var status *StatusError
	if mesh != nil || !errors.As(err, &status) || status.Status != 4 || !strings.Contains(status.Message, "simple ring") {
		t.Fatalf("crossed face returned geometry=%v error=%v", mesh, err)
	}
	mesh, err = TriangleMeshF64Polyhedron([]Vec3F64{{3, 1, 0}, {1, 1, 0}, {1, 3, 0}, {0, 3, 0}, {0, 0, 0}, {3, 0, 0}}, []uint{0, 1, 2, 3, 4, 5}, []uint{0, 6})
	if err != nil {
		t.Fatal(err)
	}
	defer mesh.Free()
	bounds, err := mesh.BoundingBoxF64()
	if err != nil || bounds == nil || bounds.mins.x != 0 || bounds.maxs.x != 3 || bounds.maxs.y != 3 {
		t.Fatalf("recovered face bounds=%v error=%v", bounds, err)
	}
}
