
$fn = 16;

//armlink();
gripper()
%base();
%translate([0,0,0.25]) rotbase();
%translate([0,0,1.5]) armlink();
%translate([0,0,4.0]) armlink();
%translate([0,0,6.5]) gripper();

module phalanx() {
    translate([0.0,0,0.25]) cube([0.1,0.2,0.5], center= true);
}

module gripper() {
    translate([0,0,0.25]) cube([0.25,0.5,0.5], center= true);
    
    translate([0,0,0.5]) { 
        cylinder(r=0.3, h=0.1, center = true);
        
        for (i = [0:3]) {
            rotate([0, 0, 120 * i])
                translate([0.4,0,0.0])
                translate([0.0,0,0.25]) {
                %cube([0.1,0.2,0.5], center= true);
            
                translate([0.0,0,0.6])
                    %cube([0.1,0.2,0.5], center= true);
            }
        }
        
    }
    
    rotate([0,90,0]) cylinder(r=0.14, h=1.0, center = true);
}

module bracket() {
    translate([0.0,0.0,0.5])
    rotate([0,90,0])
    difference() {
        hull() {
            cylinder(h=0.25, r=0.25, center= true);
            translate([0.7,0,0.0]) cube([0.5,1,0.25], center= true);
        }
        translate([0.0,0,0.0]) cylinder(h=0.3, r=0.15, center = true);
    }
}

module armlink() {
    translate([0,0,1.0]) {
        cube([0.3,0.5,2.5], center= true);
        
        translate([-0.25,0,1.0]) bracket();
        translate([0.25,0,1.0]) bracket();

    translate([0,0,-1.0]) 
        rotate([0,90,0]) 
        cylinder(r=0.14, h=1.0, center = true);
    }
}

module base() {
    translate([0,0,0.25]) cube([2,2,0.5], center= true);
}

module rotbase() {
        translate([0,0,0.25]) {
            hull() {
                cylinder(h=0.25, r=1.0);
                translate([0,0,0.25]) cube([0.25,0.25,0.25], center= true);
            }
        }

    translate([0,0,0.75]) {
        translate([-0.3,0.0,0.0]) bracket();
        translate([0.3,0.0,0.0]) bracket();
    }
}