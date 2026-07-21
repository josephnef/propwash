/*
 * CineLog35 V3 WTFPV game asset
 *
 * Clean-room, unbranded reconstruction from GEPRC's published dimensions,
 * six product photographs, and frame assembly instructions. This source
 * models the published battery-less WTFPV configuration; it is not intended
 * for fabrication. Units: millimetres. +X right, +Y front, +Z up.
 */

PART = "assembly";
QUALITY = 56;
$fn = QUALITY;

wheelbase = 142;
motor_axis = wheelbase / sqrt(2);
motor_offset = motor_axis / 2;
duct_clear_diameter = 95;
propeller_diameter = 90;
main_plate_thickness = 3.5;
main_plate_z = 14;
propeller_z = 4;
// Photographic duct-wall estimate reduced by the requested 2x while keeping
// the propeller centered vertically inside the protective cage.
duct_wall_height = 8.15;
duct_wall_min_z = propeller_z - duct_wall_height / 2;
duct_wall_max_z = propeller_z + duct_wall_height / 2;

motor_positions = [
  [-motor_offset, motor_offset],
  [ motor_offset, motor_offset],
  [-motor_offset,-motor_offset],
  [ motor_offset,-motor_offset]
];

module rounded_rect_2d(size = [10, 10], radius = 2)
{
  offset(r = radius)
    square([size[0] - radius * 2, size[1] - radius * 2], center = true);
}

module rounded_prism(size = [10, 10, 4], radius = 2)
{
  linear_extrude(height = size[2], center = true, convexity = 6)
    rounded_rect_2d([size[0], size[1]], radius);
}

module annulus(outer_radius, inner_radius, height)
{
  difference() {
    cylinder(r = outer_radius, h = height, center = true);
    cylinder(r = inner_radius, h = height + 0.5, center = true);
  }
}

module molded_duct_rail(z, upper = false)
{
  // The production guard is a hollow molded cage, not a pair of round tubes.
  // Both rails therefore use a shouldered D-section with a flat 95 mm bore,
  // a soft outer bumper, and different upper/lower tool faces.  Keeping the
  // inside edge at exactly 47.5 mm preserves the published clear diameter.
  translate([0, 0, z])
    rotate_extrude(convexity = 7, $fn = QUALITY)
      translate([duct_clear_diameter / 2, 0])
        polygon(upper ? [
          [0.00,-1.38], [3.10,-1.38], [4.05,-0.58],
          [4.05, 0.58], [3.10, 1.38], [0.00, 1.38]
        ] : [
          [0.00,-1.22], [3.18,-1.22], [4.05,-0.48],
          [4.05, 0.48], [3.18, 1.22], [0.00, 1.22]
        ]);
}

module slot_between(a, b, radius)
{
  hull() {
    translate(a) circle(r = radius);
    translate(b) circle(r = radius);
  }
}

module motor_arm_2d(p)
{
  sx = p[0] < 0 ? -1 : 1;
  sy = p[1] < 0 ? -1 : 1;
  hull() {
    translate([sx * 20.5, sy * 33.5]) circle(r = 4.6);
    translate(p) circle(r = 10.2);
  }
}

module main_plate_outline_2d()
{
  union() {
    // Irregular central chassis: tapered ends and side shoulders reproduce the
    // silhouette in the top and front official photographs.
    polygon([
      [-15,-68], [15,-68], [20,-62], [20,-52], [24,-45], [23,-31],
      [20,-23], [24,-14], [26, -4], [26, 20], [23, 29], [24, 42],
      [20, 55], [17, 67], [-17,67], [-20,55], [-24,42], [-23,29],
      [-26,20], [-26,-4], [-24,-14], [-20,-23], [-23,-31],
      [-24,-45], [-20,-52], [-20,-62]
    ]);
    for (p = motor_positions) motor_arm_2d(p);
  }
}

module main_plate_2d()
{
  difference() {
    main_plate_outline_2d();

    // Central AIO window and the asymmetric longitudinal service windows.
    rounded_rect_2d([15, 18], 2.2);
    for (y = [-47, -28, 27, 47])
      translate([0, y]) rounded_rect_2d([10.5, 13], 1.8);

    // Strap and wiring slots along the plate shoulders.
    for (x = [-18.5, 18.5], y = [-43, -16, 13, 40])
      translate([x, y]) rounded_rect_2d([3.0, 12], 1.1);

    // Distinctive chevron and triangular ventilation pattern around the FC.
    for (sx = [-1, 1]) {
      polygon([
        [sx * 5, 10], [sx * 15, 18], [sx * 12, 23], [sx * 3, 16]
      ]);
      polygon([
        [sx * 5,-10], [sx * 15,-18], [sx * 12,-23], [sx * 3,-16]
      ]);
    }

    // 25.5 mm electronics stack and four motor patterns.
    for (x = [-12.75, 12.75], y = [-12.75, 12.75])
      translate([x, y]) circle(d = 2.2);
    for (p = motor_positions) {
      translate(p) circle(d = 4.2);
      for (mx = [-6, 6], my = [-6, 6])
        translate([p[0] + mx, p[1] + my]) circle(d = 2.2);

      sx = p[0] < 0 ? -1 : 1;
      sy = p[1] < 0 ? -1 : 1;
      slot_between([sx * 28, sy * 37], [p[0] - sx * 12, p[1] - sy * 8], 1.7);
    }
  }
}

module gimbal_plate_2d()
{
  difference() {
    // Long, shallow vibration plate from assembly step 4.
    polygon([
      [-27,-6], [27,-6], [29,-2], [27,5], [10,7], [5,4],
      [-5,4], [-10,7], [-27,5], [-29,-2]
    ]);
    polygon([[-9,-2], [9,-2], [12,2], [6,4], [-6,4], [-12,2]]);
    for (x = [-23, 23], y = [-3.5, 3.5]) translate([x, y]) circle(d = 3.2);
  }
}

module carbon_frame()
{
  union() {
    translate([0, 0, main_plate_z])
      linear_extrude(height = main_plate_thickness, center = true, convexity = 10)
        main_plate_2d();

    // Narrow bottom plate from the assembly side view.
    translate([0,-2,-3.7]) rounded_prism([34, 86, 1.5], 3.5);

    // Four-point action-camera/gimbal plate at the front of the aircraft.
    translate([0, 59.5, 21.0])
      linear_extrude(height = 2.0, center = true, convexity = 5) gimbal_plate_2d();
  }
}

module cage_rib_station(angle, radius, z, tangent, radial, height)
{
  rotate([0, 0, angle]) translate([radius, 0, z]) rotate([0, 0, 90])
    rounded_prism([tangent, radial, height], min(0.72, radial * 0.28));
}

module molded_cage_rib(angle, lean = 0, broad = false)
{
  // Three stations make each perimeter rib bow tangentially and flare into
  // both rails.  Alternating lean and two widths break the toy-like repeated
  // ladder pattern while matching the injected-molded reference silhouette.
  union() {
    hull() {
      cage_rib_station(angle - lean, 49.55, duct_wall_min_z + 1.40,
        broad ? 6.8 : 5.6, broad ? 3.35 : 2.75, 2.8);
      cage_rib_station(angle,        50.15, propeller_z,
        broad ? 5.5 : 4.5, broad ? 2.95 : 2.45, 3.0);
    }
    hull() {
      cage_rib_station(angle,        50.15, propeller_z,
        broad ? 5.5 : 4.5, broad ? 2.95 : 2.45, 3.0);
      cage_rib_station(angle + lean, 49.62, duct_wall_max_z - 1.55,
        broad ? 7.0 : 5.8, broad ? 3.40 : 2.80, 3.1);
    }
  }
}

module spoke_station(radius, z, length, width, height)
{
  translate([radius, 0, z])
    rounded_prism([length, width, height], min(1.15, width * 0.24));
}

module molded_spoke(angle)
{
  // A compound, descending Y-web: broad at the motor boss, pinched through
  // the prop wash, then flared where it becomes the lower duct rail.
  rotate([0, 0, angle]) union() {
    hull() {
      spoke_station(11.4, 9.25, 5.2, 7.6, 3.5);
      spoke_station(27.5, 5.35, 7.0, 4.9, 3.2);
    }
    hull() {
      spoke_station(27.5, 5.35, 7.0, 4.9, 3.2);
      spoke_station(45.7, duct_wall_min_z + 1.18, 5.7, 8.0, 3.0);
    }
    // Shallow top rib catches highlights and reproduces the molded spine.
    hull() {
      spoke_station(13.0,11.05, 4.2, 3.2, 1.05);
      spoke_station(30.0, 6.65, 6.0, 2.0, 0.95);
    }
  }
}

module clover_motor_pad_2d()
{
  difference() {
    union() {
      circle(r = 8.8);
      for (x = [-6, 6], y = [-6, 6]) translate([x, y]) circle(r = 3.65);
    }
    circle(d = 9.0);
    for (x = [-6, 6], y = [-6, 6]) translate([x, y]) circle(d = 2.45);
  }
}

module motor_guard_boss()
{
  union() {
    // Four lobes and their 12 x 12 mm bolt pattern remain visible in the bare
    // guard while the center opening clears the motor/shaft shoulder.
    translate([0, 0, 10.72])
      linear_extrude(height = 3.65, center = true, convexity = 6)
        clover_motor_pad_2d();
    translate([0, 0, 8.55])
      linear_extrude(height = 1.65, center = true, convexity = 6)
        offset(delta = -0.65) clover_motor_pad_2d();
  }
}

module molded_corner_root(sx, sy)
{
  // Small triangular root blends the two orthogonal outboard webs into the
  // clover pad.  It is a fillet/gusset, not a third symmetric wheel spoke.
  hull() {
    translate([sx * 7.7, sy * 7.7, 9.3]) rounded_prism([4.8,4.8,3.0], 1.25);
    translate([sx * 17.0,sy * 7.8, 7.5]) rounded_prism([5.2,3.6,2.6], 0.9);
    translate([sx * 7.8, sy * 17.0,7.5]) rounded_prism([3.6,5.2,2.6], 0.9);
  }
}

module guard_one(sx, sy)
{
  union() {
    molded_duct_rail(duct_wall_min_z + 1.22, false);
    molded_duct_rail(duct_wall_max_z - 1.38, true);

    // Eight vent dividers form the long rounded rectangular wall openings.
    // The four cardinal-ish ribs are broader impact/load paths.
    for (i = [0 : 7])
      molded_cage_rib(7 + i * 45, i % 2 == 0 ? -3.4 : 2.7, i % 2 == 0);

    // Official plan views show an asymmetric L/Y web aimed into each outer
    // quadrant, leaving the two inboard sectors open.
    molded_spoke(sx > 0 ? 0 : 180);
    molded_spoke(sy > 0 ? 90 : 270);
    molded_corner_root(sx, sy);
    motor_guard_boss();
  }
}

module guard_pair_saddle(midpoint, along_x = true, inward = 1)
{
  // Organic three-point saddles reinforce each cage intersection.  The broad
  // upper blade and smaller lower heel are intentionally different, as in the
  // two tool halves visible in the assembly and side photographs.
  translate([midpoint[0], midpoint[1], 0]) union() {
    translate([0, 0, duct_wall_max_z - 1.43])
      linear_extrude(height = 3.0, center = true, convexity = 4)
        hull() {
          translate(along_x ? [-5.3, 0] : [0,-5.3]) circle(r = 2.7);
          translate(along_x ? [ 5.3, 0] : [0, 5.3]) circle(r = 2.7);
          translate(along_x ? [0,-inward * 7.0] : [-inward * 7.0,0]) circle(r = 2.0);
        }
    translate([0, 0, duct_wall_min_z + 1.35])
      linear_extrude(height = 2.7, center = true, convexity = 4)
        hull() {
          translate(along_x ? [-3.8,0] : [0,-3.8]) circle(r = 2.1);
          translate(along_x ? [ 3.8,0] : [0, 3.8]) circle(r = 2.1);
        }
  }
}

module molded_landing_foot(angle)
{
  rotate([0, 0, angle]) union() {
    hull() {
      translate([50.2,0,duct_wall_min_z + 1.18]) rounded_prism([5.0,7.2,3.0], 1.2);
      translate([52.6,0,-6.6]) rounded_prism([5.4,7.5,3.2], 1.4);
    }
    // Small outer molding lobe; it stays below the prop opening instead of
    // becoming the oversized upright block used in the earlier draft.
    hull() {
      translate([50.6,0,-3.7]) rounded_prism([4.0,5.8,2.8], 1.0);
      translate([52.0,0,-5.0]) rounded_prism([4.6,6.6,2.8], 1.15);
    }
  }
}

module prop_guards()
{
  difference() {
    union() {
      for (p = motor_positions)
        translate([p[0], p[1], 0]) guard_one(p[0] < 0 ? -1 : 1, p[1] < 0 ? -1 : 1);

      // Each official left/right molding contains its fore and aft ducts.
      for (x = [-motor_offset, motor_offset])
        guard_pair_saddle([x, 0], false, x < 0 ? -1 : 1);

      // Four integrated landing bumpers at the outside corners.
      for (p = motor_positions) {
        outward = atan2(p[1], p[0]);
        translate([p[0], p[1], 0]) molded_landing_foot(outward);
      }
    }

    // Tooling relief separates the left and right molded halves at their two
    // closest tangencies.  Both halves bolt directly through the carbon motor
    // ears and therefore remain mechanically part of the rigid assembly.
    for (y = [-motor_offset, motor_offset])
      translate([0, y, 1.5]) cube([3.0, 13.0, 24.0], center = true);
  }
}

module motor_one()
{
  union() {
    // Inverted SPEEDX2 2105.5 silhouette: plate, open stator and lower bell.
    translate([0, 0, 12.2]) cylinder(d = 20.6, h = 1.8, center = true);
    translate([0, 0, 10.2]) annulus(11.1, 9.5, 2.7);
    for (a = [0 : 45 : 315])
      rotate([0, 0, a]) translate([10.2, 0, 9.3]) rounded_prism([2.3, 1.4, 3.0], 0.45);
    translate([0, 0, 7.4]) cylinder(d1 = 21.0, d2 = 22.4, h = 3.4, center = true);
    translate([0, 0, 5.5]) cylinder(d = 20.6, h = 1.5, center = true);
    translate([0, 0, 3.9]) cylinder(d = 5.2, h = 3.5, center = true);
    translate([0, 0, 2.3]) cylinder(d = 8.6, h = 1.8, center = true, $fn = 6);
  }
}

module motors()
{
  for (p = motor_positions) translate([p[0], p[1], 0]) motor_one();
}

module copper_details()
{
  union() {
    for (p = motor_positions) {
      // Individually readable stator windings instead of a toy-like gold ring.
      for (a = [0 : 30 : 330])
        translate([p[0], p[1], 9.2]) rotate([0, 0, a]) translate([9.8, 0, 0])
          rounded_prism([3.4, 1.5, 2.2], 0.45);
      // Three short motor phase wires across each carbon arm.
      for (offset = [-0.7, 0, 0.7])
        hull() {
          translate([p[0] * 0.55 + offset, p[1] * 0.66, 12.0]) sphere(d = 0.75, $fn = 10);
          translate([p[0] * 0.82 + offset, p[1] * 0.86, 12.0]) sphere(d = 0.75, $fn = 10);
        }
    }
  }
}

module hardware()
{
  union() {
    // Motor bolts extend through carbon and the molded guard collar.
    for (p = motor_positions)
      for (mx = [-6, 6], my = [-6, 6]) {
        x = p[0] + mx;
        y = p[1] + my;
        translate([x, y, 12.2]) cylinder(d = 2.0, h = 7.2, center = true);
        translate([x, y, 16.1]) cylinder(d = 3.5, h = 1.1, center = true);
      }

    // AIO/bottom-plate textured standoffs and top retaining screws.
    for (x = [-17, 17], y = [-31, 31]) {
      translate([x, y, 4.7]) cylinder(d = 3.2, h = 17.2, center = true, $fn = 18);
      translate([x, y, 15.8]) cylinder(d = 4.0, h = 1.2, center = true);
      translate([x, y,-5.0]) cylinder(d = 4.0, h = 1.2, center = true);
    }

    for (x = [-23, 23], y = [56, 63])
      translate([x, y, 22.3]) cylinder(d = 4.2, h = 1.2, center = true);
  }
}

module pcb()
{
  union() {
    translate([0,-1,5.0]) rounded_prism([35, 35, 1.5], 2.2);
    translate([0,-34,2.0]) rounded_prism([29, 17, 1.4], 1.8);
  }
}

module components()
{
  union() {
    translate([0,-1,6.4]) rounded_prism([10, 10, 2.2], 0.9);
    for (a = [0 : 90 : 270]) rotate([0, 0, a])
      translate([13,-1,6.3]) rounded_prism([7, 3.8, 1.9], 0.5);
    for (x = [-13, -6.5, 0, 6.5, 13])
      translate([x, 15, 6.3]) rounded_prism([3.3, 5.5, 1.8], 0.45);

    // Recessed black XT60EW-M-style power socket at the rear; no bright,
    // floating battery connector is present in the published configuration.
    translate([0,-65,18]) rotate([90,0,0]) rounded_prism([15, 8, 10], 1.8);
  }
}

module aluminum_parts()
{
  union() {
    // One bent battery-retention bridge from assembly step 3. The shipping
    // photos do not contain the two broad bars used by the discarded draft.
    translate([0,-31,20.2]) rounded_prism([38, 3.2, 3.0], 0.65);
    for (x = [-17.4, 17.4])
      translate([x,-31,17.8]) rounded_prism([3.2, 8.0, 6.3], 0.65);
  }
}

module tpu_parts()
{
  union() {
    // Four large damping balls under the action-camera plate.
    for (x = [-22, 22], y = [56, 63])
      translate([x, y, 17.0]) sphere(d = 7.0, $fn = 24);

    // Empty bottom-mounted lens cradle: the official WTFPV spec includes no
    // camera, so the open bay is retained rather than filled with a fake box.
    for (x = [-10.5, 10.5])
      translate([x, 73, 3.0]) rounded_prism([4.0, 18, 13], 1.2);
    translate([0, 80,-2.5]) rounded_prism([22, 4.0, 4.0], 1.0);
    // Open front bezel and hinge bosses make the cradle read correctly from
    // the official front angle without inventing a camera that is not sold.
    difference() {
      translate([0, 82, 3.0]) rounded_prism([22, 3.0, 12], 2.0);
      translate([0, 82, 3.0]) rounded_prism([12, 4.0, 6.5], 1.3);
    }
    for (x = [-11.5, 11.5])
      translate([x, 73, 3.0]) rotate([90,0,0]) difference() {
        cylinder(d = 7.0, h = 4.2, center = true);
        cylinder(d = 2.6, h = 4.8, center = true);
      }
    for (x = [-8, 8], y = [67, 78])
      translate([x, y, 10.4]) sphere(d = 4.0, $fn = 18);

    // Rear receiver/antenna mounting bracket, intentionally empty.
    difference() {
      translate([0,-68,20]) rounded_prism([24, 9, 13], 2.5);
      translate([0,-68,20]) rounded_prism([13, 11, 6], 1.5);
    }
    for (x = [-7, 7])
      translate([x,-70,28]) difference() {
        cylinder(d = 6.5, h = 4.0, center = true);
        cylinder(d = 2.8, h = 4.6, center = true);
      }
  }
}

module blade_2d()
{
  hull() {
    translate([7, 0]) scale([1.5, 1]) circle(r = 3.8, $fn = 24);
    translate([23, 5.1]) rotate(8) scale([2.15, 0.72]) circle(r = 4.2, $fn = 24);
    translate([39.7, 8.4]) rotate(14) scale([1.45, 0.52]) circle(r = 3.5, $fn = 24);
  }
}

module propeller_shape()
{
  union() {
    cylinder(d = 13.5, h = 2.4, center = true);
    for (a = [0, 120, 240]) rotate([0, 0, a])
      linear_extrude(height = 1.5, center = true, twist = 5, slices = 6, convexity = 4)
        blade_2d();
  }
}

module propeller(clockwise = true)
{
  // Hull extrema are calibrated so the exported blade-tip diameter is 90 mm.
  prop_scale = propeller_diameter / 91.302;
  scale([prop_scale, prop_scale, 1]) {
    if (clockwise) propeller_shape();
    else mirror([0, 1, 0]) propeller_shape();
  }
}

module assembly()
{
  color([0.03,0.035,0.04]) carbon_frame();
  color([0.015,0.018,0.022]) prop_guards();
  color([0.02,0.024,0.03]) motors();
  color([0.48,0.16,0.035]) copper_details();
  color([0.08,0.09,0.10]) hardware();
  color([0.02,0.07,0.045]) pcb();
  color([0.02,0.025,0.03]) components();
  color([0.03,0.035,0.04]) aluminum_parts();
  color([0.01,0.013,0.017]) tpu_parts();
  for (i = [0 : 3]) {
    p = motor_positions[i];
    translate([p[0], p[1], propeller_z])
      color([0.035,0.04,0.05,0.9]) propeller(clockwise = i == 0 || i == 3);
  }
}

if (PART == "assembly") assembly();
else if (PART == "CarbonFrame") carbon_frame();
else if (PART == "PropGuards") prop_guards();
else if (PART == "Motors") motors();
else if (PART == "Copper") copper_details();
else if (PART == "Hardware") hardware();
else if (PART == "PCB") pcb();
else if (PART == "Components") components();
else if (PART == "Aluminum") aluminum_parts();
else if (PART == "TPU") tpu_parts();
else if (PART == "PropCW") propeller(true);
else if (PART == "PropCCW") propeller(false);
else assert(false, str("Unknown PART: ", PART));
