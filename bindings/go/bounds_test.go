package csgrs

import (
	"errors"
	"testing"
)

func TestBoundsAbsenceAndPlacement(t *testing.T) {
	cube, err := TriangleMeshF64Cube(1)
	if err != nil {
		t.Fatal(err)
	}
	defer cube.Free()
	empty, err := cube.Difference(cube)
	if err != nil {
		t.Fatal(err)
	}
	defer empty.Free()
	bounds, err := empty.BoundingBoxF64()
	if err != nil || bounds != nil {
		t.Fatalf("empty bounds: %v, %v", bounds, err)
	}
	placed, err := empty.Center()
	if err != nil {
		t.Fatal(err)
	}
	defer placed.Free()
	restored, err := placed.Union(cube)
	if err != nil {
		t.Fatal(err)
	}
	defer restored.Free()
	bounds, err = restored.BoundingBoxF64()
	if err != nil || bounds == nil {
		t.Fatalf("restored bounds: %v, %v", bounds, err)
	}
	if bounds.mins.x != 0 || bounds.maxs.x != 1 {
		t.Fatal("wrong cube bounds")
	}
	centered, err := cube.Center()
	if err != nil {
		t.Fatal(err)
	}
	defer centered.Free()
	floated, err := centered.Float()
	if err != nil {
		t.Fatal(err)
	}
	defer floated.Free()
	bounds, err = floated.BoundingBoxF64()
	if err != nil || bounds == nil {
		t.Fatalf("placed bounds: %v, %v", bounds, err)
	}
	if bounds.mins.x != -0.5 || bounds.maxs.x != 0.5 || bounds.mins.z != 0 || bounds.maxs.z != 1 {
		t.Fatal("wrong placed bounds")
	}
	square, err := CurveRegionF64Square(1)
	if err != nil {
		t.Fatal(err)
	}
	defer square.Free()
	removed, err := square.Difference(square)
	if err != nil {
		t.Fatal(err)
	}
	defer removed.Free()
	bounds, err = removed.BoundingBoxF64()
	if err != nil || bounds != nil {
		t.Fatalf("empty curve bounds: %v, %v", bounds, err)
	}
	// A failed scalar-family conversion remains a typed error, not absence.
	_, err = empty.BoundingBoxF32()
	var status *StatusError
	if !errors.As(err, &status) || status.Status != 2 || status.Message == "" {
		t.Fatalf("missing wrong-family diagnostic: %v", err)
	}
	failed, err := TriangleMeshF64Cube(-1)
	if failed != nil || !errors.As(err, &status) || status.Status != 4 {
		t.Fatalf("invalid constructor: %v, %v", failed, err)
	}
}
