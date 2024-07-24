#include "vec3d.h"
#include <math.h>
#include <stdio.h>
#include "data_formats.h"


void vec3d_print(Vec3d a) {
    //printf("<%f %f %f>\n", a.x, a.y, a.z);
}

Vec3d bmisensdata_to_vec3d(BMI2SensAxisData data) {
    return (Vec3d) {
        data.x,
        data.y,
        data.z
    };
}

BMI2SensAxisData vec3d_to_bmisensdata(Vec3d data) {
    return (BMI2SensAxisData) {
        data.x,
        data.y,
        data.z
    };
}

Vec3d vec3d_cross_product(Vec3d a, Vec3d b)
{
    return (Vec3d) {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

double vec3d_dot_product(Vec3d a, Vec3d b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double vec3d_component(Vec3d a, Vec3d b, double tolerance) {
    double magnitude_b = vec3d_magnitude(b);
    if (magnitude_b <= tolerance) {
        return 0;
    }
    return vec3d_dot_product(a, b) / magnitude_b;
}

Vec3d vec3d_add_vectors(Vec3d a, Vec3d b) {
    return (Vec3d) {
        a.x + b.x,
        a.y + b.y,
        a.z + b.z
    };
}

Vec3d vec3d_add_scalar(Vec3d a, double scalar) {
    return (Vec3d) {
        a.x + scalar,
        a.y + scalar,
        a.z + scalar
    };
}

Vec3d vec3d_multiply_scalar(Vec3d a, double scalar) {
    return (Vec3d) {
        a.x * scalar,
        a.y * scalar,
        a.z * scalar
    };
}

Vec3d vec3d_sub_vectors(Vec3d a, Vec3d b) {
    return (Vec3d) {
        a.x - b.x,
        a.y - b.y,
        a.z - b.z
    };
}

double vec3d_magnitude(Vec3d a) {
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

Vec3d vec3d_project(Vec3d a, Vec3d b) { /* a onto b*/
    Vec3d b_unit_vector = vec3d_multiply_scalar(b, 1 / vec3d_magnitude(b)); 
    double component_a_onto_b = vec3d_dot_product(a, b) / vec3d_magnitude(b);

    return vec3d_multiply_scalar(b_unit_vector, component_a_onto_b);
}

Vec3d vec3d_rotate_vector(Vec3d a, Vec3d axis, double angle) {
    Vec3d term1 = vec3d_multiply_scalar(vec3d_project(a, axis), (1 - cos(angle)));
    Vec3d term2 = vec3d_multiply_scalar(a, cos(angle));
    Vec3d term3 = vec3d_multiply_scalar(vec3d_cross_product(axis, a), sin(angle));

    return vec3d_add_vectors(term1, vec3d_add_vectors(term2, term3));
}

Vec3d vec3d_rotate_about_x(Vec3d a, double angle) {
    return (Vec3d) {
        a.x,
        a.y * cos(angle) - a.z * sin(angle),
        a.y * sin(angle) + a.z * cos(angle)
    };
}

Vec3d vec3d_rotate_about_y(Vec3d a, double angle) {
    return (Vec3d) {
        a.x * cos(angle) + a.z * sin(angle),
        a.y,
        -a.x * sin(angle) + a.z * cos(angle)
    };
}

Vec3d vec3d_rotate_about_z(Vec3d a, double angle) {
    return (Vec3d) {
        a.x * cos(angle) - a.y * sin(angle),
        a.x * sin(angle) + a.y * cos(angle),
        a.z
    };
}

Vec3d vec3d_rotate_by_rotation_vector_assuming_things_that_arent_true(Vec3d a, Vec3d rotation_vector) {
    Vec3d rotated = a;
    rotated = vec3d_rotate_about_x(rotated, rotation_vector.x);
    rotated = vec3d_rotate_about_y(rotated, rotation_vector.y);
    rotated = vec3d_rotate_about_z(rotated, rotation_vector.z);

    return rotated;
}

//double avg_orientation(vec3d a,vec3d b,vec3d c,vec3d d,vec3d e) {
Vec3d vec3d_avg_unit_5(Vec3d a,Vec3d b,Vec3d c,Vec3d d,Vec3d e) {
	
	double x = (a.x + b.x + c.x + d.x + e.x);
	double y = (a.y + b.y + c.y + d.y + e.y);
	double z = (a.z + b.z + c.z + d.z + e.z);
	
	double mag = x*x + y*y + z*z;	
	mag = sqrt(mag); 
	//I could do like the error of like all 3 axes but Idt you give af about any of them except like, down
	//return (1 - abs(y)/mag)
	return (Vec3d){x/mag, y/mag, z/mag};

}

double vec3d_vector_similarity(Vec3d a, Vec3d b) {
    return vec3d_dot_product(a, b) / (vec3d_magnitude(a) * vec3d_magnitude(b));
    // a.b = |a||b|cos(theta) => cos(theta) = a.b / |a||b|
}