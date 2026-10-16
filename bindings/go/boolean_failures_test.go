package csgrs

import (
	"errors"
	"strings"
	"testing"
)

func TestBooleanFailureCategoryAndRecovery(t *testing.T) {
	cube, err := TriangleMeshF64Cube(1)
	if err != nil {
		t.Fatal(err)
	}
	defer cube.Free()
	rotated, err := cube.RotateF64(0, 0, 13)
	if err != nil {
		t.Fatal(err)
	}
	defer rotated.Free()
	restored, err := rotated.RotateF64(0, 0, -13)
	if err != nil {
		t.Fatal(err)
	}
	defer restored.Free()
	for _, operation := range []func(*TriangleMesh) (*TriangleMesh, error){
		cube.Union, cube.Difference, cube.Intersection, cube.Xor,
	} {
		result, err := operation(restored)
		var status *StatusError
		if result != nil || !errors.As(err, &status) || status.Status != 5 ||
			!strings.Contains(status.Message, "predicate could not be decided") {
			t.Errorf("undecided Boolean returned model=%v error=%v", result, err)
		}
		if result != nil {
			result.Free()
		}
	}
	empty, err := cube.Difference(cube)
	if err != nil {
		t.Fatal(err)
	}
	defer empty.Free()
	bounds, err := empty.BoundingBoxF64()
	if err != nil || bounds != nil {
		t.Fatalf("canonical empty: %v, %v", bounds, err)
	}
	recovered, err := empty.Union(cube)
	if err != nil {
		t.Fatal(err)
	}
	defer recovered.Free()
	bounds, err = recovered.BoundingBoxF64()
	if err != nil || bounds == nil || bounds.mins.x != 0 || bounds.maxs.x != 1 {
		t.Fatalf("recovery: %v, %v", bounds, err)
	}
}
