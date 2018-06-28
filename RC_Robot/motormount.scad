$fn = 60;
large_fn = 90;
tolerance = 1.05;
height = 12;
bearing_height = 8 * tolerance;
axle_distance = 50;
stepper_size = 56;
stepper_center = 40;
bearing_OD = 29 * tolerance;
bearing_ID = 17;
hole_ID = 5.1 * tolerance;
hole_offset = 4;
stepper_tab_depth = 5;
motor_surround = 3;

length = stepper_size / 2 + bearing_OD / 2 + 
            axle_distance + hole_offset;

module hexagon(size, height) {
  boxWidth = size/1.75;
  for (r = [-60, 0, 60]) rotate([0,0,r]) cube([boxWidth, size, height], true);
}


stepper_length = 100 - stepper_tab_depth;

nut_depth = 5 * tolerance;
nut_width = 8 * tolerance;
connector_width = 10;
wall_thickness = (connector_width - nut_width) / 2;
screw_length = 35;
nut_hole_depth = screw_length - nut_width - height - stepper_tab_depth;
connector_plate_depth = 3;
connector_sep = stepper_size - 2 * hole_offset - connector_width;
wire_height = 4;
gap = 3;

module end_hole() {
    
}

module connector() {
    difference() {
        union() {
            translate([connector_width / 2, 0, connector_width / 2])
            rotate([-90])
                cylinder(stepper_length, connector_width / 2,  
        connector_width / 2);
            cube([connector_width, stepper_length, connector_width / 2]);
            cube([connector_width / 2, stepper_length, connector_width]);
        }
        
        translate([connector_width / 2, 0, connector_width / 2])
            rotate([-90])
                cylinder(nut_hole_depth + nut_depth + nut_depth, hole_ID / 2, hole_ID / 2);
        translate([connector_width / 2, stepper_length, connector_width / 2])
            rotate([90])
                cylinder(nut_hole_depth + nut_depth + nut_depth, hole_ID / 2, hole_ID / 2);
        
        translate([connector_width / 2, nut_hole_depth + nut_depth / 2, connector_width / 2])
            rotate([90, 30]) hexagon(nut_width, nut_depth);
        translate([wall_thickness, nut_hole_depth, connector_width / 2])
            cube([nut_width, nut_depth, connector_width / 2]);
        
        translate([connector_width / 2, stepper_length - nut_hole_depth - nut_depth / 2, connector_width / 2])
            rotate([90, 30]) 
                hexagon(nut_width, nut_depth);
        translate([wall_thickness, stepper_length - nut_hole_depth - nut_depth, connector_width / 2])
            cube([nut_width, nut_depth, connector_width / 2]);        
   }
    
}     

module bearing_plate() {
    difference() {
        translate([-motor_surround, -motor_surround, 0])
            cube([stepper_size + 2 * motor_surround, 
            length + 2 * motor_surround, height / 2]);
        translate([stepper_size/2, stepper_size/2+axle_distance,
                (height - bearing_height) / 2]) 
            cylinder(height, bearing_OD/2, bearing_OD/2);
        
        translate([stepper_size/2, stepper_size/2+axle_distance]) 
            cylinder(height, bearing_ID/2, bearing_ID/2);
    }
}

module half_holy_plate() {
    difference() {
        bearing_plate();
    
        translate([stepper_size-hole_offset, stepper_size - hole_offset]) 
            cylinder(height, hole_ID / 2, hole_ID / 2);
        translate([hole_offset, stepper_size - hole_offset]) 
            cylinder(height, hole_ID / 2, hole_ID / 2);
        translate([hole_offset, 
                    length - hole_offset]) 
            cylinder(height, hole_ID / 2, hole_ID / 2);
        translate([stepper_size - hole_offset, 
                    length - hole_offset]) 
            cylinder(height, hole_ID / 2, hole_ID / 2);
    }
}

module holy_plate() {
    difference() {
        half_holy_plate();
        translate([hole_offset, hole_offset]) 
            cylinder(height, hole_ID / 2, hole_ID / 2);
        translate([stepper_size - hole_offset, hole_offset]) 
            cylinder(height, hole_ID / 2, hole_ID / 2);
    }
}


module mount_plate() {

    difference() {
        holy_plate();
        translate([stepper_size / 2, stepper_size / 2]) 
            cylinder(height, stepper_center / 2, stepper_center / 2);
    }
}

module backing_plate() {
    difference() {
        half_holy_plate();
        difference() {
            cube([stepper_size, stepper_size + wire_height, height / 2]);
            cylinder(height, connector_width / 2, connector_width / 2);
            translate([stepper_size, 0])
                cylinder(height, connector_width / 2, connector_width / 2);
            translate([0, stepper_size])
                cylinder(height, connector_width / 2, connector_width / 2);
            translate([stepper_size, stepper_size])
                cylinder(height, connector_width / 2, connector_width / 2);
            
        }        
    }
}

//mount_plate(); 
//translate([stepper_size + gap + 3 * motor_surround, 0]) 
backing_plate();
//translate([0, length + gap + 3 * motor_surround]) mount_plate(); 
//translate([stepper_size + gap + 3 * motor_surround, length + gap + 3 * motor_surround]) backing_plate();
/*
translate([stepper_size + gap, length + gap]) mount_plate();
translate([0, length + gap]) mount_plate(); // */

//translate([-connector_width - gap, 0]) connector();
/*translate([-connector_width - gap, length + gap]) connector();
translate([2 * (-connector_width - gap), 0]) connector();
translate([2 * (-connector_width - gap), length + gap]) connector();

/*
translate([0, -height])
    linear_extrude(stepper_size)
    square([stepper_size, height]);

translate([-height, -height])
    linear_extrude(stepper_size)
    square([height, stepper_size + height]);
*/