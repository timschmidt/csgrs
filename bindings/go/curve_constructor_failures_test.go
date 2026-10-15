package csgrs

import (
	"errors"
	"strings"
	"testing"
)

func TestCurveConstructorFailuresAndRecovery(t *testing.T) {
	region, err := CurveRegionF64Rectangle(0, -1)
	var status *StatusError
	if region != nil || !errors.As(err, &status) || status.Status != 4 || !strings.Contains(status.Message, "height") {
		t.Fatalf("lost planar construction error: %v, %v", region, err)
	}
	region, err = CurveRegionF64Circle(0, 2)
	if region != nil || !errors.As(err, &status) || status.Status != 4 {
		t.Fatalf("invalid resolution became successful geometry: %v, %v", region, err)
	}
	region, err = CurveRegionF64Square(0)
	if err != nil {
		t.Fatal(err)
	}
	defer region.Free()
	bounds, err := region.BoundingBoxF64()
	if err != nil || bounds != nil {
		t.Fatalf("zero square is not canonical empty: %v, %v", bounds, err)
	}
	valid, err := CurveRegionF64Square(1)
	if err != nil {
		t.Fatal(err)
	}
	defer valid.Free()
	bounds, err = valid.BoundingBoxF64()
	if err != nil || bounds == nil || bounds.maxs.x != 1 {
		t.Fatalf("recovery failed: %v, %v", bounds, err)
	}
}
