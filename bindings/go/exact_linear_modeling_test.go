package csgrs

import (
	"errors"
	"strings"
	"testing"
)

func TestExactLinearExtrusionAndFailureRecovery(t *testing.T) {
	width := I128FromInt64(9007199254740993)
	region, err := CurveRegionI128Rectangle(width, I128FromInt64(1))
	if err != nil {
		t.Fatal(err)
	}
	defer region.Free()
	failed, err := region.ExtrudeI128(I128FromInt64(0))
	var status *StatusError
	if failed != nil || !errors.As(err, &status) || status.Status != 4 || !strings.Contains(status.Message, "nonzero") {
		t.Fatalf("lost invalid extrusion diagnostic: %v, %v", failed, err)
	}
	solid, err := region.ExtrudeI128(I128FromInt64(2))
	if err != nil {
		t.Fatal(err)
	}
	defer solid.Free()
	bounds, err := solid.BoundingBoxI128()
	if err != nil || bounds == nil {
		t.Fatalf("missing exact bounds: %v", err)
	}
	if bounds.maxs.x != width || bounds.maxs.y != I128FromInt64(1) || bounds.maxs.z != I128FromInt64(2) {
		t.Fatal("extrusion quantized exact integer coordinates")
	}
}
