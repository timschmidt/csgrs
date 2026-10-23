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
	// 13.1 degrees as binary64 lies in a cyclotomic field far beyond the exact
	// zero proof's budget. A rotation and its inverse still restore the cube
	// exactly, because the unit-circle relation is proved without that field.
	angle := 13.1
	rotated, err := cube.RotateF64(0, 0, angle)
	if err != nil {
		t.Fatal(err)
	}
	defer rotated.Free()
	inverse, err := rotated.RotateF64(0, 0, -angle)
	if err != nil {
		t.Fatal(err)
	}
	defer inverse.Free()
	decided, err := cube.Union(inverse)
	if err != nil {
		t.Fatalf("rotation and inverse: %v", err)
	}
	defer decided.Free()
	bounds, err := decided.BoundingBoxF64()
	if err != nil || bounds == nil || bounds.mins.x != 0 || bounds.maxs.x != 1 {
		t.Fatalf("rotation and inverse bounds: %v, %v", bounds, err)
	}
	// Undoing 13.1 and then 0.5 degrees with one rotation by their exact sum
	// needs the angle-addition identity in that field, so the restored cube's
	// coincident faces stay undecided under STRICT.
	turned, err := rotated.RotateF64(0, 0, 0.5)
	if err != nil {
		t.Fatal(err)
	}
	defer turned.Free()
	restored, err := turned.RotateF64(0, 0, -(angle + 0.5))
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
	bounds, err = empty.BoundingBoxF64()
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
