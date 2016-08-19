local ffi = require("ffi")
ffi.cdef([[typedef enum graphene_euler_order_t{GRAPHENE_EULER_ORDER_DEFAULT=-1,GRAPHENE_EULER_ORDER_XYZ=0,GRAPHENE_EULER_ORDER_YZX=1,GRAPHENE_EULER_ORDER_ZXY=2,GRAPHENE_EULER_ORDER_XZY=3,GRAPHENE_EULER_ORDER_YXZ=4,GRAPHENE_EULER_ORDER_ZYX=5};
struct graphene_simd4x4f_t {float x;float y;float z;float w;};
struct _graphene_vec2_t {float __graphene_private_value;};
struct _graphene_vec3_t {float __graphene_private_value;};
struct _graphene_vec4_t {float __graphene_private_value;};
struct _graphene_matrix_t {struct graphene_simd4x4f_t __graphene_private_value;};
struct _graphene_point_t {float x;float y;};
struct _graphene_size_t {float width;float height;};
struct _graphene_rect_t {struct _graphene_point_t origin;struct _graphene_size_t size;};
struct _graphene_point3d_t {float x;float y;float z;};
struct _graphene_quad_t {struct _graphene_point_t __graphene_private_points[4];};
struct _graphene_quaternion_t {float __graphene_private_x;float __graphene_private_y;float __graphene_private_z;float __graphene_private_w;};
struct _graphene_euler_t {struct _graphene_vec3_t __graphene_private_angles;enum graphene_euler_order_t __graphene_private_order;};
struct _graphene_plane_t {struct _graphene_vec3_t __graphene_private_normal;float __graphene_private_constant;};
struct _graphene_frustum_t {struct _graphene_plane_t __graphene_private_planes[6];};
struct _graphene_sphere_t {struct _graphene_vec3_t __graphene_private_center;float __graphene_private_radius;};
struct _graphene_box_t {struct _graphene_vec3_t __graphene_private_min;struct _graphene_vec3_t __graphene_private_max;};
struct _graphene_triangle_t {struct _graphene_vec3_t __graphene_private_a;struct _graphene_vec3_t __graphene_private_b;struct _graphene_vec3_t __graphene_private_c;};
struct _graphene_ray_t {struct _graphene_vec3_t __graphene_private_origin;struct _graphene_vec3_t __graphene_private_direction;};
_Bool(graphene_rect_equal)(const struct _graphene_rect_t*,const struct _graphene_rect_t*);
struct _graphene_plane_t*(graphene_plane_init)(struct _graphene_plane_t*,const struct _graphene_vec3_t*,float);
float(graphene_vec2_length)(const struct _graphene_vec2_t*);
_Bool(graphene_vec2_near)(const struct _graphene_vec2_t*,const struct _graphene_vec2_t*,float);
void(graphene_matrix_translate)(struct _graphene_matrix_t*,const struct _graphene_point3d_t*);
void(graphene_matrix_project_rect)(const struct _graphene_matrix_t*,const struct _graphene_rect_t*,struct _graphene_quad_t*);
struct _graphene_matrix_t*(graphene_matrix_init_identity)(struct _graphene_matrix_t*);
void(graphene_matrix_transform_point3d)(const struct _graphene_matrix_t*,const struct _graphene_point3d_t*,struct _graphene_point3d_t*);
float(graphene_simd4f_max)(const float,const float);
_Bool(graphene_simd4f_cmp_neq)(const float,const float);
_Bool(graphene_simd4f_cmp_eq)(const float,const float);
_Bool(graphene_matrix_is_identity)(const struct _graphene_matrix_t*);
struct _graphene_euler_t*(graphene_euler_init_from_vec3)(struct _graphene_euler_t*,const struct _graphene_vec3_t*,enum graphene_euler_order_t);
_Bool(graphene_ray_equal)(const struct _graphene_ray_t*,const struct _graphene_ray_t*);
void(graphene_vec3_get_xyz1)(const struct _graphene_vec3_t*,struct _graphene_vec4_t*);
struct _graphene_point3d_t*(graphene_point3d_init_from_vec3)(struct _graphene_point3d_t*,const struct _graphene_vec3_t*);
void(graphene_vec4_multiply)(const struct _graphene_vec4_t*,const struct _graphene_vec4_t*,struct _graphene_vec4_t*);
struct _graphene_triangle_t*(graphene_triangle_init_from_vec3)(struct _graphene_triangle_t*,const struct _graphene_vec3_t*,const struct _graphene_vec3_t*,const struct _graphene_vec3_t*);
const struct _graphene_point_t*(graphene_point_zero)();
struct _graphene_quad_t*(graphene_quad_init)(struct _graphene_quad_t*,const struct _graphene_point_t*,const struct _graphene_point_t*,const struct _graphene_point_t*,const struct _graphene_point_t*);
struct _graphene_quaternion_t*(graphene_quaternion_alloc)();
struct _graphene_box_t*(graphene_box_alloc)();
float(graphene_simd4f_merge_low)(const float,const float);
_Bool(graphene_vec2_equal)(const struct _graphene_vec2_t*,const struct _graphene_vec2_t*);
void(graphene_ray_get_closest_point_to_point)(const struct _graphene_ray_t*,const struct _graphene_point3d_t*,struct _graphene_point3d_t*);
struct _graphene_box_t*(graphene_box_init_from_points)(struct _graphene_box_t*,unsigned int,const struct _graphene_point3d_t*);
float(graphene_vec2_dot)(const struct _graphene_vec2_t*,const struct _graphene_vec2_t*);
float(graphene_point3d_length)(const struct _graphene_point3d_t*);
float(graphene_ray_get_distance_to_point)(const struct _graphene_ray_t*,const struct _graphene_point3d_t*);
void(graphene_ray_get_direction)(const struct _graphene_ray_t*,struct _graphene_vec3_t*);
void(graphene_ray_get_origin)(const struct _graphene_ray_t*,struct _graphene_point3d_t*);
const struct _graphene_size_t*(graphene_size_zero)();
struct _graphene_ray_t*(graphene_ray_init_from_vec3)(struct _graphene_ray_t*,const struct _graphene_vec3_t*,const struct _graphene_vec3_t*);
void(graphene_box_expand_vec3)(const struct _graphene_box_t*,const struct _graphene_vec3_t*,struct _graphene_box_t*);
struct _graphene_ray_t*(graphene_ray_init)(struct _graphene_ray_t*,const struct _graphene_point3d_t*,const struct _graphene_vec3_t*);
void(graphene_ray_free)(struct _graphene_ray_t*);
struct _graphene_ray_t*(graphene_ray_alloc)();
_Bool(graphene_triangle_equal)(const struct _graphene_triangle_t*,const struct _graphene_triangle_t*);
float(graphene_vec2_get_x)(const struct _graphene_vec2_t*);
float(graphene_simd4f_splat)(float);
_Bool(graphene_triangle_contains_point)(const struct _graphene_triangle_t*,const struct _graphene_point3d_t*);
_Bool(graphene_triangle_get_barycoords)(const struct _graphene_triangle_t*,const struct _graphene_point3d_t*,struct _graphene_vec2_t*);
void(graphene_triangle_get_bounding_box)(const struct _graphene_triangle_t*,struct _graphene_box_t*);
struct _graphene_quaternion_t*(graphene_quaternion_init_from_quaternion)(struct _graphene_quaternion_t*,const struct _graphene_quaternion_t*);
void(graphene_vec4_subtract)(const struct _graphene_vec4_t*,const struct _graphene_vec4_t*,struct _graphene_vec4_t*);
float(graphene_simd4f_get_z)(const float);
void(graphene_vec2_normalize)(const struct _graphene_vec2_t*,struct _graphene_vec2_t*);
float(graphene_triangle_get_area)(const struct _graphene_triangle_t*);
void(graphene_triangle_get_vertices)(const struct _graphene_triangle_t*,struct _graphene_vec3_t*,struct _graphene_vec3_t*,struct _graphene_vec3_t*);
void(graphene_triangle_get_points)(const struct _graphene_triangle_t*,struct _graphene_point3d_t*,struct _graphene_point3d_t*,struct _graphene_point3d_t*);
struct _graphene_triangle_t*(graphene_triangle_init_from_point3d)(struct _graphene_triangle_t*,const struct _graphene_point3d_t*,const struct _graphene_point3d_t*,const struct _graphene_point3d_t*);
void(graphene_matrix_rotate_x)(struct _graphene_matrix_t*,float);
_Bool(graphene_box_contains_box)(const struct _graphene_box_t*,const struct _graphene_box_t*);
float(graphene_simd4f_get_y)(const float);
void(graphene_vec2_free)(struct _graphene_vec2_t*);
const struct _graphene_box_t*(graphene_box_infinite)();
float(graphene_simd4f_sqrt)(const float);
void(graphene_rect_get_bottom_right)(const struct _graphene_rect_t*,struct _graphene_point_t*);
const struct _graphene_box_t*(graphene_box_one)();
void(graphene_euler_to_matrix)(const struct _graphene_euler_t*,struct _graphene_matrix_t*);
_Bool(graphene_box_equal)(const struct _graphene_box_t*,const struct _graphene_box_t*);
const struct _graphene_box_t*(graphene_box_zero)();
void(graphene_box_get_bounding_sphere)(const struct _graphene_box_t*,struct _graphene_sphere_t*);
void(graphene_box_get_vertices)(const struct _graphene_box_t*,struct _graphene_vec3_t);
void(graphene_box_get_max)(const struct _graphene_box_t*,struct _graphene_point3d_t*);
void(graphene_rect_get_top_left)(const struct _graphene_rect_t*,struct _graphene_point_t*);
void(graphene_box_get_size)(const struct _graphene_box_t*,struct _graphene_vec3_t*);
void(graphene_rect_free)(struct _graphene_rect_t*);
float(graphene_sphere_distance)(const struct _graphene_sphere_t*,const struct _graphene_point3d_t*);
float(graphene_box_get_depth)(const struct _graphene_box_t*);
float(graphene_sphere_get_radius)(const struct _graphene_sphere_t*);
float(graphene_box_get_height)(const struct _graphene_box_t*);
void(graphene_rect_offset_r)(const struct _graphene_rect_t*,float,float,struct _graphene_rect_t*);
_Bool(graphene_point3d_near)(const struct _graphene_point3d_t*,const struct _graphene_point3d_t*,float);
_Bool(graphene_point_equal)(const struct _graphene_point_t*,const struct _graphene_point_t*);
struct _graphene_vec4_t*(graphene_vec4_init_from_float)(struct _graphene_vec4_t*,const float*);
void(graphene_box_union)(const struct _graphene_box_t*,const struct _graphene_box_t*,struct _graphene_box_t*);
void(graphene_box_expand_scalar)(const struct _graphene_box_t*,float,struct _graphene_box_t*);
struct _graphene_ray_t*(graphene_ray_init_from_ray)(struct _graphene_ray_t*,const struct _graphene_ray_t*);
void(graphene_box_expand)(const struct _graphene_box_t*,const struct _graphene_point3d_t*,struct _graphene_box_t*);
void(graphene_matrix_rotate_euler)(struct _graphene_matrix_t*,const struct _graphene_euler_t*);
struct _graphene_box_t*(graphene_box_init_from_box)(struct _graphene_box_t*,const struct _graphene_box_t*);
struct _graphene_box_t*(graphene_box_init_from_vectors)(struct _graphene_box_t*,unsigned int,const struct _graphene_vec3_t*);
struct _graphene_matrix_t*(graphene_matrix_init_from_2d)(struct _graphene_matrix_t*,double,double,double,double,double,double);
struct _graphene_box_t*(graphene_box_init)(struct _graphene_box_t*,const struct _graphene_point3d_t*,const struct _graphene_point3d_t*);
void(graphene_box_free)(struct _graphene_box_t*);
void(graphene_sphere_translate)(const struct _graphene_sphere_t*,const struct _graphene_point3d_t*,struct _graphene_sphere_t*);
_Bool(graphene_sphere_is_empty)(const struct _graphene_sphere_t*);
_Bool(graphene_rect_contains_rect)(const struct _graphene_rect_t*,const struct _graphene_rect_t*);
_Bool(graphene_sphere_contains_point)(const struct _graphene_sphere_t*,const struct _graphene_point3d_t*);
_Bool(graphene_sphere_equal)(const struct _graphene_sphere_t*,const struct _graphene_sphere_t*);
void(graphene_sphere_get_bounding_box)(const struct _graphene_sphere_t*,struct _graphene_box_t*);
void(graphene_sphere_get_center)(const struct _graphene_sphere_t*,struct _graphene_point3d_t*);
struct _graphene_sphere_t*(graphene_sphere_init_from_vectors)(struct _graphene_sphere_t*,unsigned int,const struct _graphene_vec3_t*,const struct _graphene_point3d_t*);
struct _graphene_sphere_t*(graphene_sphere_init_from_points)(struct _graphene_sphere_t*,unsigned int,const struct _graphene_point3d_t*,const struct _graphene_point3d_t*);
struct _graphene_sphere_t*(graphene_sphere_init)(struct _graphene_sphere_t*,const struct _graphene_point3d_t*,float);
void(graphene_quaternion_to_vec4)(const struct _graphene_quaternion_t*,struct _graphene_vec4_t*);
void(graphene_sphere_free)(struct _graphene_sphere_t*);
struct _graphene_sphere_t*(graphene_sphere_alloc)();
void(graphene_frustum_get_planes)(const struct _graphene_frustum_t*,struct _graphene_plane_t);
void(graphene_euler_reorder)(const struct _graphene_euler_t*,enum graphene_euler_order_t,struct _graphene_euler_t*);
_Bool(graphene_frustum_intersects_box)(const struct _graphene_frustum_t*,const struct _graphene_box_t*);
void(graphene_vec4_divide)(const struct _graphene_vec4_t*,const struct _graphene_vec4_t*,struct _graphene_vec4_t*);
_Bool(graphene_frustum_contains_point)(const struct _graphene_frustum_t*,const struct _graphene_point3d_t*);
struct _graphene_frustum_t*(graphene_frustum_init_from_matrix)(struct _graphene_frustum_t*,const struct _graphene_matrix_t*);
struct _graphene_matrix_t*(graphene_matrix_init_from_float)(struct _graphene_matrix_t*,const float*);
void(graphene_frustum_free)(struct _graphene_frustum_t*);
struct _graphene_frustum_t*(graphene_frustum_alloc)();
float(graphene_plane_get_constant)(const struct _graphene_plane_t*);
void(graphene_plane_get_normal)(const struct _graphene_plane_t*,struct _graphene_vec3_t*);
float(graphene_plane_distance)(const struct _graphene_plane_t*,const struct _graphene_point3d_t*);
void(graphene_vec3_negate)(const struct _graphene_vec3_t*,struct _graphene_vec3_t*);
void(graphene_plane_negate)(const struct _graphene_plane_t*,struct _graphene_plane_t*);
const struct _graphene_vec4_t*(graphene_vec4_y_axis)();
struct _graphene_plane_t*(graphene_plane_init_from_points)(struct _graphene_plane_t*,const struct _graphene_point3d_t*,const struct _graphene_point3d_t*,const struct _graphene_point3d_t*);
void(graphene_triangle_free)(struct _graphene_triangle_t*);
struct _graphene_plane_t*(graphene_plane_init_from_plane)(struct _graphene_plane_t*,const struct _graphene_plane_t*);
struct _graphene_vec3_t*(graphene_vec3_init_from_float)(struct _graphene_vec3_t*,const float*);
float(graphene_euler_get_y)(const struct _graphene_euler_t*);
float(graphene_simd4f_reciprocal)(const float);
void(graphene_vec3_add)(const struct _graphene_vec3_t*,const struct _graphene_vec3_t*,struct _graphene_vec3_t*);
_Bool(graphene_box_contains_point)(const struct _graphene_box_t*,const struct _graphene_point3d_t*);
void(graphene_euler_to_vec3)(const struct _graphene_euler_t*,struct _graphene_vec3_t*);
float(graphene_simd4f_dot3_scalar)(const float,const float);
void(graphene_vec3_get_xy0)(const struct _graphene_vec3_t*,struct _graphene_vec3_t*);
void(graphene_vec2_add)(const struct _graphene_vec2_t*,const struct _graphene_vec2_t*,struct _graphene_vec2_t*);
void(graphene_plane_free)(struct _graphene_plane_t*);
float(graphene_euler_get_x)(const struct _graphene_euler_t*);
struct _graphene_size_t*(graphene_size_init)(struct _graphene_size_t*,float,float);
void(graphene_rect_get_center)(const struct _graphene_rect_t*,struct _graphene_point_t*);
struct _graphene_matrix_t*(graphene_matrix_alloc)();
struct _graphene_euler_t*(graphene_euler_init_with_order)(struct _graphene_euler_t*,float,float,float,enum graphene_euler_order_t);
struct _graphene_euler_t*(graphene_euler_init_from_quaternion)(struct _graphene_euler_t*,const struct _graphene_quaternion_t*,enum graphene_euler_order_t);
struct _graphene_euler_t*(graphene_euler_init_from_matrix)(struct _graphene_euler_t*,const struct _graphene_matrix_t*,enum graphene_euler_order_t);
struct _graphene_euler_t*(graphene_euler_init_from_euler)(struct _graphene_euler_t*,const struct _graphene_euler_t*);
struct _graphene_vec4_t*(graphene_vec4_init_from_vec2)(struct _graphene_vec4_t*,const struct _graphene_vec2_t*,float,float);
struct _graphene_matrix_t*(graphene_matrix_init_from_vec4)(struct _graphene_matrix_t*,const struct _graphene_vec4_t*,const struct _graphene_vec4_t*,const struct _graphene_vec4_t*,const struct _graphene_vec4_t*);
void(graphene_euler_free)(struct _graphene_euler_t*);
struct _graphene_euler_t*(graphene_euler_alloc)();
float(graphene_rect_get_y)(const struct _graphene_rect_t*);
void(graphene_quaternion_invert)(const struct _graphene_quaternion_t*,struct _graphene_quaternion_t*);
void(graphene_quaternion_normalize)(const struct _graphene_quaternion_t*,struct _graphene_quaternion_t*);
float(graphene_simd4f_init_2f)(const float*);
void(graphene_quaternion_to_radians)(const struct _graphene_quaternion_t*,float*,float*,float*);
void(graphene_quaternion_to_angle_vec3)(const struct _graphene_quaternion_t*,float*,struct _graphene_vec3_t*);
void(graphene_matrix_get_row)(const struct _graphene_matrix_t*,unsigned int,struct _graphene_vec4_t*);
_Bool(graphene_quaternion_equal)(const struct _graphene_quaternion_t*,const struct _graphene_quaternion_t*);
void(graphene_matrix_transform_vec4)(const struct _graphene_matrix_t*,const struct _graphene_vec4_t*,struct _graphene_vec4_t*);
float(graphene_simd4f_mul)(const float,const float);
void(graphene_quaternion_to_angles)(const struct _graphene_quaternion_t*,float*,float*,float*);
struct _graphene_size_t*(graphene_size_alloc)();
struct _graphene_matrix_t*(graphene_matrix_init_rotate)(struct _graphene_matrix_t*,float,const struct _graphene_vec3_t*);
struct _graphene_quaternion_t*(graphene_quaternion_init_from_angle_vec3)(struct _graphene_quaternion_t*,float,const struct _graphene_vec3_t*);
struct _graphene_vec2_t*(graphene_vec2_alloc)();
struct _graphene_quaternion_t*(graphene_quaternion_init_from_radians)(struct _graphene_quaternion_t*,float,float,float);
void(graphene_point_to_vec2)(const struct _graphene_point_t*,struct _graphene_vec2_t*);
struct _graphene_quaternion_t*(graphene_quaternion_init_from_matrix)(struct _graphene_quaternion_t*,const struct _graphene_matrix_t*);
struct _graphene_quaternion_t*(graphene_quaternion_init_from_vec4)(struct _graphene_quaternion_t*,const struct _graphene_vec4_t*);
void(graphene_triangle_get_plane)(const struct _graphene_triangle_t*,struct _graphene_plane_t*);
struct _graphene_quaternion_t*(graphene_quaternion_init_identity)(struct _graphene_quaternion_t*);
struct _graphene_quaternion_t*(graphene_quaternion_init)(struct _graphene_quaternion_t*,float,float,float,float);
void(graphene_quaternion_free)(struct _graphene_quaternion_t*);
const struct _graphene_point_t*(graphene_quad_get_point)(const struct _graphene_quad_t*,unsigned int);
void(graphene_vec3_free)(struct _graphene_vec3_t*);
void(graphene_matrix_unproject_point3d)(const struct _graphene_matrix_t*,const struct _graphene_matrix_t*,const struct _graphene_point3d_t*,struct _graphene_point3d_t*);
struct _graphene_quad_t*(graphene_quad_init_from_rect)(struct _graphene_quad_t*,const struct _graphene_rect_t*);
struct _graphene_matrix_t*(graphene_matrix_init_frustum)(struct _graphene_matrix_t*,float,float,float,float,float,float);
void(graphene_vec2_multiply)(const struct _graphene_vec2_t*,const struct _graphene_vec2_t*,struct _graphene_vec2_t*);
float(graphene_point_distance)(const struct _graphene_point_t*,const struct _graphene_point_t*,float*,float*);
void(graphene_matrix_project_point)(const struct _graphene_matrix_t*,const struct _graphene_point_t*,struct _graphene_point_t*);
const struct _graphene_point3d_t*(graphene_point3d_zero)();
void(graphene_point3d_normalize_viewport)(const struct _graphene_point3d_t*,const struct _graphene_rect_t*,float,float,struct _graphene_point3d_t*);
float(graphene_vec3_get_z)(const struct _graphene_vec3_t*);
float(graphene_point3d_distance)(const struct _graphene_point3d_t*,const struct _graphene_point3d_t*,struct _graphene_vec3_t*);
void(graphene_point3d_normalize)(const struct _graphene_point3d_t*,struct _graphene_point3d_t*);
float(graphene_point3d_dot)(const struct _graphene_point3d_t*,const struct _graphene_point3d_t*);
_Bool(graphene_simd4f_cmp_le)(const float,const float);
void(graphene_rect_get_bottom_left)(const struct _graphene_rect_t*,struct _graphene_point_t*);
void(graphene_point3d_scale)(const struct _graphene_point3d_t*,float,struct _graphene_point3d_t*);
_Bool(graphene_point3d_equal)(const struct _graphene_point3d_t*,const struct _graphene_point3d_t*);
void(graphene_point3d_to_vec3)(const struct _graphene_point3d_t*,struct _graphene_vec3_t*);
struct _graphene_point3d_t*(graphene_point3d_init)(struct _graphene_point3d_t*,float,float,float);
void(graphene_matrix_untransform_bounds)(const struct _graphene_matrix_t*,const struct _graphene_rect_t*,const struct _graphene_rect_t*,struct _graphene_rect_t*);
void(graphene_matrix_transpose)(const struct _graphene_matrix_t*,struct _graphene_matrix_t*);
_Bool(graphene_vec3_equal)(const struct _graphene_vec3_t*,const struct _graphene_vec3_t*);
struct _graphene_point3d_t*(graphene_point3d_alloc)();
void(graphene_rect_expand)(const struct _graphene_rect_t*,const struct _graphene_point_t*,struct _graphene_rect_t*);
void(graphene_rect_interpolate)(const struct _graphene_rect_t*,const struct _graphene_rect_t*,double,struct _graphene_rect_t*);
float(graphene_simd4f_cross3)(const float,const float);
void(graphene_rect_round)(const struct _graphene_rect_t*,struct _graphene_rect_t*);
_Bool(graphene_simd4f_cmp_lt)(const float,const float);
void(graphene_rect_inset_r)(const struct _graphene_rect_t*,float,float,struct _graphene_rect_t*);
void(graphene_box_get_center)(const struct _graphene_box_t*,struct _graphene_point3d_t*);
_Bool(graphene_box_intersection)(const struct _graphene_box_t*,const struct _graphene_box_t*,struct _graphene_box_t*);
_Bool(graphene_rect_contains_point)(const struct _graphene_rect_t*,const struct _graphene_point_t*);
struct _graphene_matrix_t*(graphene_matrix_init_from_matrix)(struct _graphene_matrix_t*,const struct _graphene_matrix_t*);
_Bool(graphene_rect_intersection)(const struct _graphene_rect_t*,const struct _graphene_rect_t*,struct _graphene_rect_t*);
void(graphene_rect_union)(const struct _graphene_rect_t*,const struct _graphene_rect_t*,struct _graphene_rect_t*);
_Bool(graphene_vec3_near)(const struct _graphene_vec3_t*,const struct _graphene_vec3_t*,float);
void(graphene_size_interpolate)(const struct _graphene_size_t*,const struct _graphene_size_t*,double,struct _graphene_size_t*);
const struct _graphene_vec4_t*(graphene_vec4_z_axis)();
struct _graphene_matrix_t*(graphene_matrix_init_scale)(struct _graphene_matrix_t*,float,float,float);
void(graphene_rect_get_top_right)(const struct _graphene_rect_t*,struct _graphene_point_t*);
void(graphene_box_get_min)(const struct _graphene_box_t*,struct _graphene_point3d_t*);
void(graphene_rect_normalize_r)(const struct _graphene_rect_t*,struct _graphene_rect_t*);
struct _graphene_rect_t*(graphene_rect_normalize)(struct _graphene_rect_t*);
struct _graphene_rect_t*(graphene_rect_init_from_rect)(struct _graphene_rect_t*,const struct _graphene_rect_t*);
struct _graphene_rect_t*(graphene_rect_init)(struct _graphene_rect_t*,float,float,float,float);
const struct _graphene_box_t*(graphene_box_minus_one)();
_Bool(graphene_size_equal)(const struct _graphene_size_t*,const struct _graphene_size_t*);
void(graphene_quad_bounds)(const struct _graphene_quad_t*,struct _graphene_rect_t*);
_Bool(graphene_euler_equal)(const struct _graphene_euler_t*,const struct _graphene_euler_t*);
void(graphene_size_free)(struct _graphene_size_t*);
void(graphene_point_interpolate)(const struct _graphene_point_t*,const struct _graphene_point_t*,double,struct _graphene_point_t*);
struct _graphene_matrix_t*(graphene_matrix_init_look_at)(struct _graphene_matrix_t*,const struct _graphene_vec3_t*,const struct _graphene_vec3_t*,const struct _graphene_vec3_t*);
void(graphene_quad_free)(struct _graphene_quad_t*);
struct _graphene_point_t*(graphene_point_init_from_vec2)(struct _graphene_point_t*,const struct _graphene_vec2_t*);
void(graphene_vec4_free)(struct _graphene_vec4_t*);
struct _graphene_point_t*(graphene_point_init)(struct _graphene_point_t*,float,float);
void(graphene_point_free)(struct _graphene_point_t*);
struct _graphene_point_t*(graphene_point_alloc)();
void(graphene_matrix_print)(const struct _graphene_matrix_t*);
void(graphene_matrix_interpolate)(const struct _graphene_matrix_t*,const struct _graphene_matrix_t*,double,struct _graphene_matrix_t*);
float(graphene_matrix_get_z_scale)(const struct _graphene_matrix_t*);
float(graphene_matrix_get_y_scale)(const struct _graphene_matrix_t*);
void(graphene_matrix_perspective)(const struct _graphene_matrix_t*,float,struct _graphene_matrix_t*);
void(graphene_vec3_cross)(const struct _graphene_vec3_t*,const struct _graphene_vec3_t*,struct _graphene_vec3_t*);
float(graphene_quaternion_dot)(const struct _graphene_quaternion_t*,const struct _graphene_quaternion_t*);
void(graphene_matrix_skew_xy)(struct _graphene_matrix_t*,float);
struct _graphene_frustum_t*(graphene_frustum_init_from_frustum)(struct _graphene_frustum_t*,const struct _graphene_frustum_t*);
void(graphene_matrix_skew_xz)(struct _graphene_matrix_t*,float);
void(graphene_matrix_scale)(struct _graphene_matrix_t*,float,float,float);
struct _graphene_box_t*(graphene_box_init_from_vec3)(struct _graphene_box_t*,const struct _graphene_vec3_t*,const struct _graphene_vec3_t*);
void(graphene_matrix_rotate_z)(struct _graphene_matrix_t*,float);
void(graphene_matrix_rotate_y)(struct _graphene_matrix_t*,float);
struct _graphene_plane_t*(graphene_plane_init_from_point)(struct _graphene_plane_t*,const struct _graphene_vec3_t*,const struct _graphene_point3d_t*);
void(graphene_quaternion_slerp)(const struct _graphene_quaternion_t*,const struct _graphene_quaternion_t*,float,struct _graphene_quaternion_t*);
struct _graphene_quad_t*(graphene_quad_alloc)();
void(graphene_matrix_free)(struct _graphene_matrix_t*);
void(graphene_matrix_transform_box)(const struct _graphene_matrix_t*,const struct _graphene_box_t*,struct _graphene_box_t*);
void(graphene_matrix_transform_sphere)(const struct _graphene_matrix_t*,const struct _graphene_sphere_t*,struct _graphene_sphere_t*);
void(graphene_matrix_to_float)(const struct _graphene_matrix_t*,float*);
void(graphene_matrix_transform_point)(const struct _graphene_matrix_t*,const struct _graphene_point_t*,struct _graphene_point_t*);
void(graphene_matrix_transform_vec3)(const struct _graphene_matrix_t*,const struct _graphene_vec3_t*,struct _graphene_vec3_t*);
float(graphene_matrix_determinant)(const struct _graphene_matrix_t*);
void(graphene_matrix_multiply)(const struct _graphene_matrix_t*,const struct _graphene_matrix_t*,struct _graphene_matrix_t*);
_Bool(graphene_point_near)(const struct _graphene_point_t*,const struct _graphene_point_t*,float);
_Bool(graphene_matrix_is_backface_visible)(const struct _graphene_matrix_t*);
_Bool(graphene_matrix_is_singular)(const struct _graphene_matrix_t*);
_Bool(graphene_matrix_to_2d)(const struct _graphene_matrix_t*,double*,double*,double*,double*,double*,double*);
float(graphene_simd4f_div)(const float,const float);
struct _graphene_matrix_t*(graphene_matrix_init_translate)(struct _graphene_matrix_t*,const struct _graphene_point3d_t*);
struct _graphene_matrix_t*(graphene_matrix_init_skew)(struct _graphene_matrix_t*,float,float);
float(graphene_matrix_get_value)(const struct _graphene_matrix_t*,unsigned int,unsigned int);
struct _graphene_matrix_t*(graphene_matrix_init_ortho)(struct _graphene_matrix_t*,float,float,float,float,float,float);
struct _graphene_matrix_t*(graphene_matrix_init_perspective)(struct _graphene_matrix_t*,float,float,float,float);
struct _graphene_frustum_t*(graphene_frustum_init)(struct _graphene_frustum_t*,const struct _graphene_plane_t*,const struct _graphene_plane_t*,const struct _graphene_plane_t*,const struct _graphene_plane_t*,const struct _graphene_plane_t*,const struct _graphene_plane_t*);
void(graphene_matrix_transform_ray)(const struct _graphene_matrix_t*,const struct _graphene_ray_t*,struct _graphene_ray_t*);
const struct _graphene_vec4_t*(graphene_vec4_w_axis)();
void(graphene_rect_get_vertices)(const struct _graphene_rect_t*,struct _graphene_vec2_t);
void(graphene_plane_normalize)(const struct _graphene_plane_t*,struct _graphene_plane_t*);
const struct _graphene_vec4_t*(graphene_vec4_x_axis)();
const struct _graphene_vec4_t*(graphene_vec4_one)();
const struct _graphene_vec4_t*(graphene_vec4_zero)();
float(graphene_rect_get_x)(const struct _graphene_rect_t*);
void(graphene_vec4_get_xy)(const struct _graphene_vec4_t*,struct _graphene_vec2_t*);
float(graphene_vec4_get_w)(const struct _graphene_vec4_t*);
float(graphene_simd4f_dot3)(const float,const float);
float(graphene_vec4_get_z)(const struct _graphene_vec4_t*);
float(graphene_vec4_get_y)(const struct _graphene_vec4_t*);
float(graphene_vec4_get_x)(const struct _graphene_vec4_t*);
void(graphene_vec4_max)(const struct _graphene_vec4_t*,const struct _graphene_vec4_t*,struct _graphene_vec4_t*);
void(graphene_vec4_min)(const struct _graphene_vec4_t*,const struct _graphene_vec4_t*,struct _graphene_vec4_t*);
_Bool(graphene_vec4_near)(const struct _graphene_vec4_t*,const struct _graphene_vec4_t*,float);
_Bool(graphene_vec4_equal)(const struct _graphene_vec4_t*,const struct _graphene_vec4_t*);
void(graphene_vec4_negate)(const struct _graphene_vec4_t*,struct _graphene_vec4_t*);
void(graphene_vec4_scale)(const struct _graphene_vec4_t*,float,struct _graphene_vec4_t*);
float(graphene_vec4_length)(const struct _graphene_vec4_t*);
float(graphene_vec4_dot)(const struct _graphene_vec4_t*,const struct _graphene_vec4_t*);
_Bool(graphene_frustum_intersects_sphere)(const struct _graphene_frustum_t*,const struct _graphene_sphere_t*);
void(graphene_triangle_get_normal)(const struct _graphene_triangle_t*,struct _graphene_vec3_t*);
void(graphene_vec4_add)(const struct _graphene_vec4_t*,const struct _graphene_vec4_t*,struct _graphene_vec4_t*);
void(graphene_vec4_to_float)(const struct _graphene_vec4_t*,float*);
struct _graphene_euler_t*(graphene_euler_init)(struct _graphene_euler_t*,float,float,float);
struct _graphene_vec4_t*(graphene_vec4_init_from_vec3)(struct _graphene_vec4_t*,const struct _graphene_vec3_t*,float);
struct _graphene_vec4_t*(graphene_vec4_init_from_vec4)(struct _graphene_vec4_t*,const struct _graphene_vec4_t*);
struct _graphene_vec4_t*(graphene_vec4_init)(struct _graphene_vec4_t*,float,float,float,float);
struct _graphene_point_t*(graphene_point_init_from_point)(struct _graphene_point_t*,const struct _graphene_point_t*);
struct _graphene_vec4_t*(graphene_vec4_alloc)();
const struct _graphene_vec3_t*(graphene_vec3_z_axis)();
const struct _graphene_vec3_t*(graphene_vec3_y_axis)();
const struct _graphene_vec3_t*(graphene_vec3_x_axis)();
const struct _graphene_vec3_t*(graphene_vec3_one)();
const struct _graphene_vec3_t*(graphene_vec3_zero)();
void(graphene_vec3_get_xyzw)(const struct _graphene_vec3_t*,float,struct _graphene_vec4_t*);
float(graphene_euler_get_z)(const struct _graphene_euler_t*);
float(graphene_simd4f_splat_y)(const float);
void(graphene_point3d_interpolate)(const struct _graphene_point3d_t*,const struct _graphene_point3d_t*,double,struct _graphene_point3d_t*);
float(graphene_vec3_get_y)(const struct _graphene_vec3_t*);
float(graphene_vec3_get_x)(const struct _graphene_vec3_t*);
void(graphene_vec3_max)(const struct _graphene_vec3_t*,const struct _graphene_vec3_t*,struct _graphene_vec3_t*);
void(graphene_vec3_min)(const struct _graphene_vec3_t*,const struct _graphene_vec3_t*,struct _graphene_vec3_t*);
float(graphene_rect_get_height)(const struct _graphene_rect_t*);
void(graphene_point3d_free)(struct _graphene_point3d_t*);
_Bool(graphene_plane_equal)(const struct _graphene_plane_t*,const struct _graphene_plane_t*);
void(graphene_vec3_scale)(const struct _graphene_vec3_t*,float,struct _graphene_vec3_t*);
void(graphene_vec3_normalize)(const struct _graphene_vec3_t*,struct _graphene_vec3_t*);
float(graphene_vec3_length)(const struct _graphene_vec3_t*);
float(graphene_vec3_dot)(const struct _graphene_vec3_t*,const struct _graphene_vec3_t*);
void(graphene_vec3_divide)(const struct _graphene_vec3_t*,const struct _graphene_vec3_t*,struct _graphene_vec3_t*);
void(graphene_vec3_multiply)(const struct _graphene_vec3_t*,const struct _graphene_vec3_t*,struct _graphene_vec3_t*);
void(graphene_vec3_subtract)(const struct _graphene_vec3_t*,const struct _graphene_vec3_t*,struct _graphene_vec3_t*);
void(graphene_vec2_negate)(const struct _graphene_vec2_t*,struct _graphene_vec2_t*);
struct _graphene_plane_t*(graphene_plane_alloc)();
void(graphene_vec3_to_float)(const struct _graphene_vec3_t*,float*);
struct _graphene_vec3_t*(graphene_vec3_init_from_vec3)(struct _graphene_vec3_t*,const struct _graphene_vec3_t*);
struct _graphene_vec3_t*(graphene_vec3_init)(struct _graphene_vec3_t*,float,float,float);
struct _graphene_size_t*(graphene_size_init_from_size)(struct _graphene_size_t*,const struct _graphene_size_t*);
struct _graphene_vec3_t*(graphene_vec3_alloc)();
float(graphene_simd4f_neg)(const float);
const struct _graphene_vec2_t*(graphene_vec2_y_axis)();
const struct _graphene_vec2_t*(graphene_vec2_x_axis)();
const struct _graphene_vec2_t*(graphene_vec2_one)();
const struct _graphene_vec2_t*(graphene_vec2_zero)();
float(graphene_vec2_get_y)(const struct _graphene_vec2_t*);
float(graphene_simd4f_shuffle_wxyz)(const float);
void(graphene_vec2_min)(const struct _graphene_vec2_t*,const struct _graphene_vec2_t*,struct _graphene_vec2_t*);
void(graphene_vec2_scale)(const struct _graphene_vec2_t*,float,struct _graphene_vec2_t*);
void(graphene_vec2_divide)(const struct _graphene_vec2_t*,const struct _graphene_vec2_t*,struct _graphene_vec2_t*);
void(graphene_vec2_subtract)(const struct _graphene_vec2_t*,const struct _graphene_vec2_t*,struct _graphene_vec2_t*);
void(graphene_point3d_cross)(const struct _graphene_point3d_t*,const struct _graphene_point3d_t*,struct _graphene_point3d_t*);
struct _graphene_vec2_t*(graphene_vec2_init_from_float)(struct _graphene_vec2_t*,const float*);
struct _graphene_vec2_t*(graphene_vec2_init_from_vec2)(struct _graphene_vec2_t*,const struct _graphene_vec2_t*);
const struct _graphene_box_t*(graphene_box_empty)();
float(graphene_simd4f_flip_sign_1010)(const float);
_Bool(graphene_simd4f_cmp_ge)(const float,const float);
void(graphene_vec2_to_float)(const struct _graphene_vec2_t*,float*);
void(graphene_simd4x4f_transpose_in_place)(struct graphene_simd4x4f_t*);
float(graphene_simd4f_flip_sign_0101)(const float);
float(graphene_simd4f_merge_w)(const float,float);
float(graphene_simd4f_zero_zw)(const float);
float(graphene_simd4f_zero_w)(const float);
float(graphene_simd4f_shuffle_yzwx)(const float);
float(graphene_simd4f_rsqrt)(const float);
float(graphene_simd4f_shuffle_zwxy)(const float);
float(graphene_simd4f_get)(const float,unsigned int);
void(graphene_vec2_max)(const struct _graphene_vec2_t*,const struct _graphene_vec2_t*,struct _graphene_vec2_t*);
float(graphene_simd4f_sub)(const float,const float);
float(graphene_simd4f_get_w)(const float);
float(graphene_simd4f_add)(const float,const float);
float(graphene_simd4f_splat_z)(const float);
void(graphene_vec3_get_xy)(const struct _graphene_vec3_t*,struct _graphene_vec2_t*);
_Bool(graphene_simd4f_cmp_gt)(const float,const float);
void(graphene_triangle_get_midpoint)(const struct _graphene_triangle_t*,struct _graphene_point3d_t*);
void(graphene_simd4f_dup_2f)(const float,float*);
void(graphene_simd4f_dup_3f)(const float,float*);
void(graphene_simd4f_dup_4f)(const float,float*);
float(graphene_simd4f_init_4f)(const float*);
float(graphene_simd4f_init_zero)();
float(graphene_simd4f_init)(float,float,float,float);
struct _graphene_vec2_t*(graphene_vec2_init)(struct _graphene_vec2_t*,float,float);
_Bool(graphene_quad_contains)(const struct _graphene_quad_t*,const struct _graphene_point_t*);
void(graphene_matrix_project_rect_bounds)(const struct _graphene_matrix_t*,const struct _graphene_rect_t*,struct _graphene_rect_t*);
struct _graphene_rect_t*(graphene_rect_inset)(struct _graphene_rect_t*,float,float);
struct _graphene_rect_t*(graphene_rect_round_to_pixel)(struct _graphene_rect_t*);
enum graphene_euler_order_t(graphene_euler_get_order)(const struct _graphene_euler_t*);
void(graphene_size_scale)(const struct _graphene_size_t*,float,struct _graphene_size_t*);
const struct _graphene_rect_t*(graphene_rect_zero)();
void(graphene_vec4_get_xyz)(const struct _graphene_vec4_t*,struct _graphene_vec3_t*);
void(graphene_matrix_rotate_quaternion)(struct _graphene_matrix_t*,const struct _graphene_quaternion_t*);
void(graphene_matrix_skew_yz)(struct _graphene_matrix_t*,float);
float(graphene_simd4f_merge_high)(const float,const float);
struct _graphene_triangle_t*(graphene_triangle_alloc)();
struct _graphene_quaternion_t*(graphene_quaternion_init_from_euler)(struct _graphene_quaternion_t*,const struct _graphene_euler_t*);
struct _graphene_rect_t*(graphene_rect_offset)(struct _graphene_rect_t*,float,float);
float(graphene_simd4f_init_3f)(const float*);
struct _graphene_point3d_t*(graphene_point3d_init_from_point)(struct _graphene_point3d_t*,const struct _graphene_point3d_t*);
struct _graphene_plane_t*(graphene_plane_init_from_vec4)(struct _graphene_plane_t*,const struct _graphene_vec4_t*);
struct _graphene_rect_t*(graphene_rect_alloc)();
void(graphene_matrix_rotate)(struct _graphene_matrix_t*,float,const struct _graphene_vec3_t*);
float(graphene_simd4f_splat_w)(const float);
float(graphene_rect_get_width)(const struct _graphene_rect_t*);
_Bool(graphene_matrix_untransform_point)(const struct _graphene_matrix_t*,const struct _graphene_point_t*,const struct _graphene_rect_t*,struct _graphene_point_t*);
float(graphene_ray_get_distance_to_plane)(const struct _graphene_ray_t*,const struct _graphene_plane_t*);
float(graphene_simd4f_min)(const float,const float);
void(graphene_vec3_get_xyz0)(const struct _graphene_vec3_t*,struct _graphene_vec4_t*);
_Bool(graphene_matrix_is_2d)(const struct _graphene_matrix_t*);
struct _graphene_quaternion_t*(graphene_quaternion_init_from_angles)(struct _graphene_quaternion_t*,float,float,float);
void(graphene_matrix_transform_rect)(const struct _graphene_matrix_t*,const struct _graphene_rect_t*,struct _graphene_quad_t*);
float(graphene_simd4f_get_x)(const float);
_Bool(graphene_matrix_inverse)(const struct _graphene_matrix_t*,struct _graphene_matrix_t*);
struct _graphene_quad_t*(graphene_quad_init_from_points)(struct _graphene_quad_t*,const struct _graphene_point_t);
float(graphene_simd4f_splat_x)(const float);
void(graphene_ray_get_position_at)(const struct _graphene_ray_t*,float,struct _graphene_point3d_t*);
void(graphene_quaternion_to_matrix)(const struct _graphene_quaternion_t*,struct _graphene_matrix_t*);
float(graphene_box_get_width)(const struct _graphene_box_t*);
void(graphene_vec4_normalize)(const struct _graphene_vec4_t*,struct _graphene_vec4_t*);
void(graphene_matrix_transform_bounds)(const struct _graphene_matrix_t*,const struct _graphene_rect_t*,struct _graphene_rect_t*);
float(graphene_matrix_get_x_scale)(const struct _graphene_matrix_t*);
const struct _graphene_box_t*(graphene_box_one_minus_one)();
void(graphene_matrix_normalize)(const struct _graphene_matrix_t*,struct _graphene_matrix_t*);
]])
local CLIB = ffi.load(_G.FFI_LIB or "graphene")
local library = {}


--====helper metatables====
	local metatables = {}
	local object_cache = {}

	local function wrap_pointer(ptr, meta_name)
		-- TODO
		-- you should be able to use cdata as key and it would use the address
		-- but apparently that doesn't work
		local id = tostring(ptr)

		if not object_cache[meta_name] then
			object_cache[meta_name] = setmetatable({}, {__mode = "v"})
		end

		if not object_cache[meta_name][id] then
			object_cache[meta_name][id] = setmetatable({ptr = ptr}, metatables[meta_name])
		end

		return object_cache[meta_name][id]
	end
--====helper metatables====

library = {
	RectEqual = CLIB.graphene_rect_equal,
	PlaneInit = CLIB.graphene_plane_init,
	Vec2Length = CLIB.graphene_vec2_length,
	Vec2Near = CLIB.graphene_vec2_near,
	MatrixTranslate = CLIB.graphene_matrix_translate,
	MatrixProjectRect = CLIB.graphene_matrix_project_rect,
	MatrixInitIdentity = CLIB.graphene_matrix_init_identity,
	MatrixTransformPoint3d = CLIB.graphene_matrix_transform_point3d,
	Simd4fMax = CLIB.graphene_simd4f_max,
	Simd4fCmpNeq = CLIB.graphene_simd4f_cmp_neq,
	Simd4fCmpEq = CLIB.graphene_simd4f_cmp_eq,
	MatrixIsIdentity = CLIB.graphene_matrix_is_identity,
	EulerInitFromVec3 = CLIB.graphene_euler_init_from_vec3,
	RayEqual = CLIB.graphene_ray_equal,
	Vec3GetXyz1 = CLIB.graphene_vec3_get_xyz1,
	Point3dInitFromVec3 = CLIB.graphene_point3d_init_from_vec3,
	Vec4Multiply = CLIB.graphene_vec4_multiply,
	TriangleInitFromVec3 = CLIB.graphene_triangle_init_from_vec3,
	PointZero = CLIB.graphene_point_zero,
	QuadInit = CLIB.graphene_quad_init,
	QuaternionAlloc = CLIB.graphene_quaternion_alloc,
	BoxAlloc = CLIB.graphene_box_alloc,
	Simd4fMergeLow = CLIB.graphene_simd4f_merge_low,
	Vec2Equal = CLIB.graphene_vec2_equal,
	RayGetClosestPointToPoint = CLIB.graphene_ray_get_closest_point_to_point,
	BoxInitFromPoints = CLIB.graphene_box_init_from_points,
	Vec2Dot = CLIB.graphene_vec2_dot,
	Point3dLength = CLIB.graphene_point3d_length,
	RayGetDistanceToPoint = CLIB.graphene_ray_get_distance_to_point,
	RayGetDirection = CLIB.graphene_ray_get_direction,
	RayGetOrigin = CLIB.graphene_ray_get_origin,
	SizeZero = CLIB.graphene_size_zero,
	RayInitFromVec3 = CLIB.graphene_ray_init_from_vec3,
	BoxExpandVec3 = CLIB.graphene_box_expand_vec3,
	RayInit = CLIB.graphene_ray_init,
	RayFree = CLIB.graphene_ray_free,
	RayAlloc = CLIB.graphene_ray_alloc,
	TriangleEqual = CLIB.graphene_triangle_equal,
	Vec2GetX = CLIB.graphene_vec2_get_x,
	Simd4fSplat = CLIB.graphene_simd4f_splat,
	TriangleContainsPoint = CLIB.graphene_triangle_contains_point,
	TriangleGetBarycoords = CLIB.graphene_triangle_get_barycoords,
	TriangleGetBoundingBox = CLIB.graphene_triangle_get_bounding_box,
	QuaternionInitFromQuaternion = CLIB.graphene_quaternion_init_from_quaternion,
	Vec4Subtract = CLIB.graphene_vec4_subtract,
	Simd4fGetZ = CLIB.graphene_simd4f_get_z,
	Vec2Normalize = CLIB.graphene_vec2_normalize,
	TriangleGetArea = CLIB.graphene_triangle_get_area,
	TriangleGetVertices = CLIB.graphene_triangle_get_vertices,
	TriangleGetPoints = CLIB.graphene_triangle_get_points,
	TriangleInitFromPoint3d = CLIB.graphene_triangle_init_from_point3d,
	MatrixRotateX = CLIB.graphene_matrix_rotate_x,
	BoxContainsBox = CLIB.graphene_box_contains_box,
	Simd4fGetY = CLIB.graphene_simd4f_get_y,
	Vec2Free = CLIB.graphene_vec2_free,
	BoxInfinite = CLIB.graphene_box_infinite,
	Simd4fSqrt = CLIB.graphene_simd4f_sqrt,
	RectGetBottomRight = CLIB.graphene_rect_get_bottom_right,
	BoxOne = CLIB.graphene_box_one,
	EulerToMatrix = CLIB.graphene_euler_to_matrix,
	BoxEqual = CLIB.graphene_box_equal,
	BoxZero = CLIB.graphene_box_zero,
	BoxGetBoundingSphere = CLIB.graphene_box_get_bounding_sphere,
	BoxGetVertices = CLIB.graphene_box_get_vertices,
	BoxGetMax = CLIB.graphene_box_get_max,
	RectGetTopLeft = CLIB.graphene_rect_get_top_left,
	BoxGetSize = CLIB.graphene_box_get_size,
	RectFree = CLIB.graphene_rect_free,
	SphereDistance = CLIB.graphene_sphere_distance,
	BoxGetDepth = CLIB.graphene_box_get_depth,
	SphereGetRadius = CLIB.graphene_sphere_get_radius,
	BoxGetHeight = CLIB.graphene_box_get_height,
	RectOffsetR = CLIB.graphene_rect_offset_r,
	Point3dNear = CLIB.graphene_point3d_near,
	PointEqual = CLIB.graphene_point_equal,
	Vec4InitFromFloat = CLIB.graphene_vec4_init_from_float,
	BoxUnion = CLIB.graphene_box_union,
	BoxExpandScalar = CLIB.graphene_box_expand_scalar,
	RayInitFromRay = CLIB.graphene_ray_init_from_ray,
	BoxExpand = CLIB.graphene_box_expand,
	MatrixRotateEuler = CLIB.graphene_matrix_rotate_euler,
	BoxInitFromBox = CLIB.graphene_box_init_from_box,
	BoxInitFromVectors = CLIB.graphene_box_init_from_vectors,
	MatrixInitFrom_2d = CLIB.graphene_matrix_init_from_2d,
	BoxInit = CLIB.graphene_box_init,
	BoxFree = CLIB.graphene_box_free,
	SphereTranslate = CLIB.graphene_sphere_translate,
	SphereIsEmpty = CLIB.graphene_sphere_is_empty,
	RectContainsRect = CLIB.graphene_rect_contains_rect,
	SphereContainsPoint = CLIB.graphene_sphere_contains_point,
	SphereEqual = CLIB.graphene_sphere_equal,
	SphereGetBoundingBox = CLIB.graphene_sphere_get_bounding_box,
	SphereGetCenter = CLIB.graphene_sphere_get_center,
	SphereInitFromVectors = CLIB.graphene_sphere_init_from_vectors,
	SphereInitFromPoints = CLIB.graphene_sphere_init_from_points,
	SphereInit = CLIB.graphene_sphere_init,
	QuaternionToVec4 = CLIB.graphene_quaternion_to_vec4,
	SphereFree = CLIB.graphene_sphere_free,
	SphereAlloc = CLIB.graphene_sphere_alloc,
	FrustumGetPlanes = CLIB.graphene_frustum_get_planes,
	EulerReorder = CLIB.graphene_euler_reorder,
	FrustumIntersectsBox = CLIB.graphene_frustum_intersects_box,
	Vec4Divide = CLIB.graphene_vec4_divide,
	FrustumContainsPoint = CLIB.graphene_frustum_contains_point,
	FrustumInitFromMatrix = CLIB.graphene_frustum_init_from_matrix,
	MatrixInitFromFloat = CLIB.graphene_matrix_init_from_float,
	FrustumFree = CLIB.graphene_frustum_free,
	FrustumAlloc = CLIB.graphene_frustum_alloc,
	PlaneGetConstant = CLIB.graphene_plane_get_constant,
	PlaneGetNormal = CLIB.graphene_plane_get_normal,
	PlaneDistance = CLIB.graphene_plane_distance,
	Vec3Negate = CLIB.graphene_vec3_negate,
	PlaneNegate = CLIB.graphene_plane_negate,
	Vec4YAxis = CLIB.graphene_vec4_y_axis,
	PlaneInitFromPoints = CLIB.graphene_plane_init_from_points,
	TriangleFree = CLIB.graphene_triangle_free,
	PlaneInitFromPlane = CLIB.graphene_plane_init_from_plane,
	Vec3InitFromFloat = CLIB.graphene_vec3_init_from_float,
	EulerGetY = CLIB.graphene_euler_get_y,
	Simd4fReciprocal = CLIB.graphene_simd4f_reciprocal,
	Vec3Add = CLIB.graphene_vec3_add,
	BoxContainsPoint = CLIB.graphene_box_contains_point,
	EulerToVec3 = CLIB.graphene_euler_to_vec3,
	Simd4fDot3Scalar = CLIB.graphene_simd4f_dot3_scalar,
	Vec3GetXy0 = CLIB.graphene_vec3_get_xy0,
	Vec2Add = CLIB.graphene_vec2_add,
	PlaneFree = CLIB.graphene_plane_free,
	EulerGetX = CLIB.graphene_euler_get_x,
	SizeInit = CLIB.graphene_size_init,
	RectGetCenter = CLIB.graphene_rect_get_center,
	MatrixAlloc = CLIB.graphene_matrix_alloc,
	EulerInitWithOrder = CLIB.graphene_euler_init_with_order,
	EulerInitFromQuaternion = CLIB.graphene_euler_init_from_quaternion,
	EulerInitFromMatrix = CLIB.graphene_euler_init_from_matrix,
	EulerInitFromEuler = CLIB.graphene_euler_init_from_euler,
	Vec4InitFromVec2 = CLIB.graphene_vec4_init_from_vec2,
	MatrixInitFromVec4 = CLIB.graphene_matrix_init_from_vec4,
	EulerFree = CLIB.graphene_euler_free,
	EulerAlloc = CLIB.graphene_euler_alloc,
	RectGetY = CLIB.graphene_rect_get_y,
	QuaternionInvert = CLIB.graphene_quaternion_invert,
	QuaternionNormalize = CLIB.graphene_quaternion_normalize,
	Simd4fInit_2f = CLIB.graphene_simd4f_init_2f,
	QuaternionToRadians = CLIB.graphene_quaternion_to_radians,
	QuaternionToAngleVec3 = CLIB.graphene_quaternion_to_angle_vec3,
	MatrixGetRow = CLIB.graphene_matrix_get_row,
	QuaternionEqual = CLIB.graphene_quaternion_equal,
	MatrixTransformVec4 = CLIB.graphene_matrix_transform_vec4,
	Simd4fMul = CLIB.graphene_simd4f_mul,
	QuaternionToAngles = CLIB.graphene_quaternion_to_angles,
	SizeAlloc = CLIB.graphene_size_alloc,
	MatrixInitRotate = CLIB.graphene_matrix_init_rotate,
	QuaternionInitFromAngleVec3 = CLIB.graphene_quaternion_init_from_angle_vec3,
	Vec2Alloc = CLIB.graphene_vec2_alloc,
	QuaternionInitFromRadians = CLIB.graphene_quaternion_init_from_radians,
	PointToVec2 = CLIB.graphene_point_to_vec2,
	QuaternionInitFromMatrix = CLIB.graphene_quaternion_init_from_matrix,
	QuaternionInitFromVec4 = CLIB.graphene_quaternion_init_from_vec4,
	TriangleGetPlane = CLIB.graphene_triangle_get_plane,
	QuaternionInitIdentity = CLIB.graphene_quaternion_init_identity,
	QuaternionInit = CLIB.graphene_quaternion_init,
	QuaternionFree = CLIB.graphene_quaternion_free,
	QuadGetPoint = CLIB.graphene_quad_get_point,
	Vec3Free = CLIB.graphene_vec3_free,
	MatrixUnprojectPoint3d = CLIB.graphene_matrix_unproject_point3d,
	QuadInitFromRect = CLIB.graphene_quad_init_from_rect,
	MatrixInitFrustum = CLIB.graphene_matrix_init_frustum,
	Vec2Multiply = CLIB.graphene_vec2_multiply,
	PointDistance = CLIB.graphene_point_distance,
	MatrixProjectPoint = CLIB.graphene_matrix_project_point,
	Point3dZero = CLIB.graphene_point3d_zero,
	Point3dNormalizeViewport = CLIB.graphene_point3d_normalize_viewport,
	Vec3GetZ = CLIB.graphene_vec3_get_z,
	Point3dDistance = CLIB.graphene_point3d_distance,
	Point3dNormalize = CLIB.graphene_point3d_normalize,
	Point3dDot = CLIB.graphene_point3d_dot,
	Simd4fCmpLe = CLIB.graphene_simd4f_cmp_le,
	RectGetBottomLeft = CLIB.graphene_rect_get_bottom_left,
	Point3dScale = CLIB.graphene_point3d_scale,
	Point3dEqual = CLIB.graphene_point3d_equal,
	Point3dToVec3 = CLIB.graphene_point3d_to_vec3,
	Point3dInit = CLIB.graphene_point3d_init,
	MatrixUntransformBounds = CLIB.graphene_matrix_untransform_bounds,
	MatrixTranspose = CLIB.graphene_matrix_transpose,
	Vec3Equal = CLIB.graphene_vec3_equal,
	Point3dAlloc = CLIB.graphene_point3d_alloc,
	RectExpand = CLIB.graphene_rect_expand,
	RectInterpolate = CLIB.graphene_rect_interpolate,
	Simd4fCross3 = CLIB.graphene_simd4f_cross3,
	RectRound = CLIB.graphene_rect_round,
	Simd4fCmpLt = CLIB.graphene_simd4f_cmp_lt,
	RectInsetR = CLIB.graphene_rect_inset_r,
	BoxGetCenter = CLIB.graphene_box_get_center,
	BoxIntersection = CLIB.graphene_box_intersection,
	RectContainsPoint = CLIB.graphene_rect_contains_point,
	MatrixInitFromMatrix = CLIB.graphene_matrix_init_from_matrix,
	RectIntersection = CLIB.graphene_rect_intersection,
	RectUnion = CLIB.graphene_rect_union,
	Vec3Near = CLIB.graphene_vec3_near,
	SizeInterpolate = CLIB.graphene_size_interpolate,
	Vec4ZAxis = CLIB.graphene_vec4_z_axis,
	MatrixInitScale = CLIB.graphene_matrix_init_scale,
	RectGetTopRight = CLIB.graphene_rect_get_top_right,
	BoxGetMin = CLIB.graphene_box_get_min,
	RectNormalizeR = CLIB.graphene_rect_normalize_r,
	RectNormalize = CLIB.graphene_rect_normalize,
	RectInitFromRect = CLIB.graphene_rect_init_from_rect,
	RectInit = CLIB.graphene_rect_init,
	BoxMinusOne = CLIB.graphene_box_minus_one,
	SizeEqual = CLIB.graphene_size_equal,
	QuadBounds = CLIB.graphene_quad_bounds,
	EulerEqual = CLIB.graphene_euler_equal,
	SizeFree = CLIB.graphene_size_free,
	PointInterpolate = CLIB.graphene_point_interpolate,
	MatrixInitLookAt = CLIB.graphene_matrix_init_look_at,
	QuadFree = CLIB.graphene_quad_free,
	PointInitFromVec2 = CLIB.graphene_point_init_from_vec2,
	Vec4Free = CLIB.graphene_vec4_free,
	PointInit = CLIB.graphene_point_init,
	PointFree = CLIB.graphene_point_free,
	PointAlloc = CLIB.graphene_point_alloc,
	MatrixPrint = CLIB.graphene_matrix_print,
	MatrixInterpolate = CLIB.graphene_matrix_interpolate,
	MatrixGetZScale = CLIB.graphene_matrix_get_z_scale,
	MatrixGetYScale = CLIB.graphene_matrix_get_y_scale,
	MatrixPerspective = CLIB.graphene_matrix_perspective,
	Vec3Cross = CLIB.graphene_vec3_cross,
	QuaternionDot = CLIB.graphene_quaternion_dot,
	MatrixSkewXy = CLIB.graphene_matrix_skew_xy,
	FrustumInitFromFrustum = CLIB.graphene_frustum_init_from_frustum,
	MatrixSkewXz = CLIB.graphene_matrix_skew_xz,
	MatrixScale = CLIB.graphene_matrix_scale,
	BoxInitFromVec3 = CLIB.graphene_box_init_from_vec3,
	MatrixRotateZ = CLIB.graphene_matrix_rotate_z,
	MatrixRotateY = CLIB.graphene_matrix_rotate_y,
	PlaneInitFromPoint = CLIB.graphene_plane_init_from_point,
	QuaternionSlerp = CLIB.graphene_quaternion_slerp,
	QuadAlloc = CLIB.graphene_quad_alloc,
	MatrixFree = CLIB.graphene_matrix_free,
	MatrixTransformBox = CLIB.graphene_matrix_transform_box,
	MatrixTransformSphere = CLIB.graphene_matrix_transform_sphere,
	MatrixToFloat = CLIB.graphene_matrix_to_float,
	MatrixTransformPoint = CLIB.graphene_matrix_transform_point,
	MatrixTransformVec3 = CLIB.graphene_matrix_transform_vec3,
	MatrixDeterminant = CLIB.graphene_matrix_determinant,
	MatrixMultiply = CLIB.graphene_matrix_multiply,
	PointNear = CLIB.graphene_point_near,
	MatrixIsBackfaceVisible = CLIB.graphene_matrix_is_backface_visible,
	MatrixIsSingular = CLIB.graphene_matrix_is_singular,
	MatrixTo_2d = CLIB.graphene_matrix_to_2d,
	Simd4fDiv = CLIB.graphene_simd4f_div,
	MatrixInitTranslate = CLIB.graphene_matrix_init_translate,
	MatrixInitSkew = CLIB.graphene_matrix_init_skew,
	MatrixGetValue = CLIB.graphene_matrix_get_value,
	MatrixInitOrtho = CLIB.graphene_matrix_init_ortho,
	MatrixInitPerspective = CLIB.graphene_matrix_init_perspective,
	FrustumInit = CLIB.graphene_frustum_init,
	MatrixTransformRay = CLIB.graphene_matrix_transform_ray,
	Vec4WAxis = CLIB.graphene_vec4_w_axis,
	RectGetVertices = CLIB.graphene_rect_get_vertices,
	PlaneNormalize = CLIB.graphene_plane_normalize,
	Vec4XAxis = CLIB.graphene_vec4_x_axis,
	Vec4One = CLIB.graphene_vec4_one,
	Vec4Zero = CLIB.graphene_vec4_zero,
	RectGetX = CLIB.graphene_rect_get_x,
	Vec4GetXy = CLIB.graphene_vec4_get_xy,
	Vec4GetW = CLIB.graphene_vec4_get_w,
	Simd4fDot3 = CLIB.graphene_simd4f_dot3,
	Vec4GetZ = CLIB.graphene_vec4_get_z,
	Vec4GetY = CLIB.graphene_vec4_get_y,
	Vec4GetX = CLIB.graphene_vec4_get_x,
	Vec4Max = CLIB.graphene_vec4_max,
	Vec4Min = CLIB.graphene_vec4_min,
	Vec4Near = CLIB.graphene_vec4_near,
	Vec4Equal = CLIB.graphene_vec4_equal,
	Vec4Negate = CLIB.graphene_vec4_negate,
	Vec4Scale = CLIB.graphene_vec4_scale,
	Vec4Length = CLIB.graphene_vec4_length,
	Vec4Dot = CLIB.graphene_vec4_dot,
	FrustumIntersectsSphere = CLIB.graphene_frustum_intersects_sphere,
	TriangleGetNormal = CLIB.graphene_triangle_get_normal,
	Vec4Add = CLIB.graphene_vec4_add,
	Vec4ToFloat = CLIB.graphene_vec4_to_float,
	EulerInit = CLIB.graphene_euler_init,
	Vec4InitFromVec3 = CLIB.graphene_vec4_init_from_vec3,
	Vec4InitFromVec4 = CLIB.graphene_vec4_init_from_vec4,
	Vec4Init = CLIB.graphene_vec4_init,
	PointInitFromPoint = CLIB.graphene_point_init_from_point,
	Vec4Alloc = CLIB.graphene_vec4_alloc,
	Vec3ZAxis = CLIB.graphene_vec3_z_axis,
	Vec3YAxis = CLIB.graphene_vec3_y_axis,
	Vec3XAxis = CLIB.graphene_vec3_x_axis,
	Vec3One = CLIB.graphene_vec3_one,
	Vec3Zero = CLIB.graphene_vec3_zero,
	Vec3GetXyzw = CLIB.graphene_vec3_get_xyzw,
	EulerGetZ = CLIB.graphene_euler_get_z,
	Simd4fSplatY = CLIB.graphene_simd4f_splat_y,
	Point3dInterpolate = CLIB.graphene_point3d_interpolate,
	Vec3GetY = CLIB.graphene_vec3_get_y,
	Vec3GetX = CLIB.graphene_vec3_get_x,
	Vec3Max = CLIB.graphene_vec3_max,
	Vec3Min = CLIB.graphene_vec3_min,
	RectGetHeight = CLIB.graphene_rect_get_height,
	Point3dFree = CLIB.graphene_point3d_free,
	PlaneEqual = CLIB.graphene_plane_equal,
	Vec3Scale = CLIB.graphene_vec3_scale,
	Vec3Normalize = CLIB.graphene_vec3_normalize,
	Vec3Length = CLIB.graphene_vec3_length,
	Vec3Dot = CLIB.graphene_vec3_dot,
	Vec3Divide = CLIB.graphene_vec3_divide,
	Vec3Multiply = CLIB.graphene_vec3_multiply,
	Vec3Subtract = CLIB.graphene_vec3_subtract,
	Vec2Negate = CLIB.graphene_vec2_negate,
	PlaneAlloc = CLIB.graphene_plane_alloc,
	Vec3ToFloat = CLIB.graphene_vec3_to_float,
	Vec3InitFromVec3 = CLIB.graphene_vec3_init_from_vec3,
	Vec3Init = CLIB.graphene_vec3_init,
	SizeInitFromSize = CLIB.graphene_size_init_from_size,
	Vec3Alloc = CLIB.graphene_vec3_alloc,
	Simd4fNeg = CLIB.graphene_simd4f_neg,
	Vec2YAxis = CLIB.graphene_vec2_y_axis,
	Vec2XAxis = CLIB.graphene_vec2_x_axis,
	Vec2One = CLIB.graphene_vec2_one,
	Vec2Zero = CLIB.graphene_vec2_zero,
	Vec2GetY = CLIB.graphene_vec2_get_y,
	Simd4fShuffleWxyz = CLIB.graphene_simd4f_shuffle_wxyz,
	Vec2Min = CLIB.graphene_vec2_min,
	Vec2Scale = CLIB.graphene_vec2_scale,
	Vec2Divide = CLIB.graphene_vec2_divide,
	Vec2Subtract = CLIB.graphene_vec2_subtract,
	Point3dCross = CLIB.graphene_point3d_cross,
	Vec2InitFromFloat = CLIB.graphene_vec2_init_from_float,
	Vec2InitFromVec2 = CLIB.graphene_vec2_init_from_vec2,
	BoxEmpty = CLIB.graphene_box_empty,
	Simd4fFlipSign_1010 = CLIB.graphene_simd4f_flip_sign_1010,
	Simd4fCmpGe = CLIB.graphene_simd4f_cmp_ge,
	Vec2ToFloat = CLIB.graphene_vec2_to_float,
	Simd4x4fTransposeInPlace = CLIB.graphene_simd4x4f_transpose_in_place,
	Simd4fFlipSign_0101 = CLIB.graphene_simd4f_flip_sign_0101,
	Simd4fMergeW = CLIB.graphene_simd4f_merge_w,
	Simd4fZeroZw = CLIB.graphene_simd4f_zero_zw,
	Simd4fZeroW = CLIB.graphene_simd4f_zero_w,
	Simd4fShuffleYzwx = CLIB.graphene_simd4f_shuffle_yzwx,
	Simd4fRsqrt = CLIB.graphene_simd4f_rsqrt,
	Simd4fShuffleZwxy = CLIB.graphene_simd4f_shuffle_zwxy,
	Simd4fGet = CLIB.graphene_simd4f_get,
	Vec2Max = CLIB.graphene_vec2_max,
	Simd4fSub = CLIB.graphene_simd4f_sub,
	Simd4fGetW = CLIB.graphene_simd4f_get_w,
	Simd4fAdd = CLIB.graphene_simd4f_add,
	Simd4fSplatZ = CLIB.graphene_simd4f_splat_z,
	Vec3GetXy = CLIB.graphene_vec3_get_xy,
	Simd4fCmpGt = CLIB.graphene_simd4f_cmp_gt,
	TriangleGetMidpoint = CLIB.graphene_triangle_get_midpoint,
	Simd4fDup_2f = CLIB.graphene_simd4f_dup_2f,
	Simd4fDup_3f = CLIB.graphene_simd4f_dup_3f,
	Simd4fDup_4f = CLIB.graphene_simd4f_dup_4f,
	Simd4fInit_4f = CLIB.graphene_simd4f_init_4f,
	Simd4fInitZero = CLIB.graphene_simd4f_init_zero,
	Simd4fInit = CLIB.graphene_simd4f_init,
	Vec2Init = CLIB.graphene_vec2_init,
	QuadContains = CLIB.graphene_quad_contains,
	MatrixProjectRectBounds = CLIB.graphene_matrix_project_rect_bounds,
	RectInset = CLIB.graphene_rect_inset,
	RectRoundToPixel = CLIB.graphene_rect_round_to_pixel,
	EulerGetOrder = CLIB.graphene_euler_get_order,
	SizeScale = CLIB.graphene_size_scale,
	RectZero = CLIB.graphene_rect_zero,
	Vec4GetXyz = CLIB.graphene_vec4_get_xyz,
	MatrixRotateQuaternion = CLIB.graphene_matrix_rotate_quaternion,
	MatrixSkewYz = CLIB.graphene_matrix_skew_yz,
	Simd4fMergeHigh = CLIB.graphene_simd4f_merge_high,
	TriangleAlloc = CLIB.graphene_triangle_alloc,
	QuaternionInitFromEuler = CLIB.graphene_quaternion_init_from_euler,
	RectOffset = CLIB.graphene_rect_offset,
	Simd4fInit_3f = CLIB.graphene_simd4f_init_3f,
	Point3dInitFromPoint = CLIB.graphene_point3d_init_from_point,
	PlaneInitFromVec4 = CLIB.graphene_plane_init_from_vec4,
	RectAlloc = CLIB.graphene_rect_alloc,
	MatrixRotate = CLIB.graphene_matrix_rotate,
	Simd4fSplatW = CLIB.graphene_simd4f_splat_w,
	RectGetWidth = CLIB.graphene_rect_get_width,
	MatrixUntransformPoint = CLIB.graphene_matrix_untransform_point,
	RayGetDistanceToPlane = CLIB.graphene_ray_get_distance_to_plane,
	Simd4fMin = CLIB.graphene_simd4f_min,
	Vec3GetXyz0 = CLIB.graphene_vec3_get_xyz0,
	MatrixIs_2d = CLIB.graphene_matrix_is_2d,
	QuaternionInitFromAngles = CLIB.graphene_quaternion_init_from_angles,
	MatrixTransformRect = CLIB.graphene_matrix_transform_rect,
	Simd4fGetX = CLIB.graphene_simd4f_get_x,
	MatrixInverse = CLIB.graphene_matrix_inverse,
	QuadInitFromPoints = CLIB.graphene_quad_init_from_points,
	Simd4fSplatX = CLIB.graphene_simd4f_splat_x,
	RayGetPositionAt = CLIB.graphene_ray_get_position_at,
	QuaternionToMatrix = CLIB.graphene_quaternion_to_matrix,
	BoxGetWidth = CLIB.graphene_box_get_width,
	Vec4Normalize = CLIB.graphene_vec4_normalize,
	MatrixTransformBounds = CLIB.graphene_matrix_transform_bounds,
	MatrixGetXScale = CLIB.graphene_matrix_get_x_scale,
	BoxOneMinusOne = CLIB.graphene_box_one_minus_one,
	MatrixNormalize = CLIB.graphene_matrix_normalize,
}
library.e = {
	EULER_ORDER_DEFAULT = ffi.cast("enum graphene_euler_order_t", "GRAPHENE_EULER_ORDER_DEFAULT"),
	EULER_ORDER_XYZ = ffi.cast("enum graphene_euler_order_t", "GRAPHENE_EULER_ORDER_XYZ"),
	EULER_ORDER_YZX = ffi.cast("enum graphene_euler_order_t", "GRAPHENE_EULER_ORDER_YZX"),
	EULER_ORDER_ZXY = ffi.cast("enum graphene_euler_order_t", "GRAPHENE_EULER_ORDER_ZXY"),
	EULER_ORDER_XZY = ffi.cast("enum graphene_euler_order_t", "GRAPHENE_EULER_ORDER_XZY"),
	EULER_ORDER_YXZ = ffi.cast("enum graphene_euler_order_t", "GRAPHENE_EULER_ORDER_YXZ"),
	EULER_ORDER_ZYX = ffi.cast("enum graphene_euler_order_t", "GRAPHENE_EULER_ORDER_ZYX"),
}
library.ray = library.RayAlloc
library.triangle = library.TriangleAlloc
library.plane = library.PlaneAlloc
local float_t = ffi.typeof('float[3]')
library.vec3 = function(_1, _2, _3) local s = library.Vec3Alloc() s:InitFromFloat(float_t(_1 or 0, _2 or 0, _3 or 0)) return s end
library.sphere = library.SphereAlloc
library.box = library.BoxAlloc
library.quaternion = library.QuaternionAlloc
library.quad = library.QuadAlloc
library.point = library.PointAlloc
local float_t = ffi.typeof('float[16]')
library.matrix = function(_1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16) local s = library.MatrixAlloc() s:InitFromFloat(float_t(_1 or 0, _2 or 0, _3 or 0, _4 or 1, _5 or 0, _6 or 0, _7 or 1, _8 or 0, _9 or 0, _10 or 1, _11 or 0, _12 or 0, _13 or 1, _14 or 0, _15 or 0, _16 or 0)) return s end
library.euler = library.EulerAlloc
local float_t = ffi.typeof('float[2]')
library.vec2 = function(_1, _2) local s = library.Vec2Alloc() s:InitFromFloat(float_t(_1 or 0, _2 or 0)) return s end
local float_t = ffi.typeof('float[4]')
library.vec4 = function(_1, _2, _3, _4) local s = library.Vec4Alloc() s:InitFromFloat(float_t(_1 or 0, _2 or 0, _3 or 0, _4 or 0)) return s end
library.size = library.SizeAlloc
library.point3d = function(x,y,z) local s = library.Point3dAlloc() s:Init(x or 0, y or 0, z or 0) return s end
library.frustum = library.FrustumAlloc
library.rect = library.RectAlloc
do
	local META = {
		GetXyz = CLIB.graphene_vec4_get_xyz,
		GetW = CLIB.graphene_vec4_get_w,
		InitFromVec3 = CLIB.graphene_vec4_init_from_vec3,
		__mul = CLIB.graphene_vec4_multiply,
		Equal = CLIB.graphene_vec4_equal,
		Free = CLIB.graphene_vec4_free,
		__sub = CLIB.graphene_vec4_subtract,
		Add = CLIB.graphene_vec4_add,
		Init = CLIB.graphene_vec4_init,
		__unm = CLIB.graphene_vec4_negate,
		Subtract = CLIB.graphene_vec4_subtract,
		GetZ = CLIB.graphene_vec4_get_z,
		InitFromVec4 = CLIB.graphene_vec4_init_from_vec4,
		__gc = CLIB.graphene_vec4_free,
		GetX = CLIB.graphene_vec4_get_x,
		Length = CLIB.graphene_vec4_length,
		__div = CLIB.graphene_vec4_divide,
		Divide = CLIB.graphene_vec4_divide,
		__len = CLIB.graphene_vec4_length,
		Multiply = CLIB.graphene_vec4_multiply,
		Dot = CLIB.graphene_vec4_dot,
		GetXy = CLIB.graphene_vec4_get_xy,
		ToFloat = CLIB.graphene_vec4_to_float,
		Min = CLIB.graphene_vec4_min,
		Scale = CLIB.graphene_vec4_scale,
		Near = CLIB.graphene_vec4_near,
		Max = CLIB.graphene_vec4_max,
		__add = CLIB.graphene_vec4_add,
		InitFromFloat = CLIB.graphene_vec4_init_from_float,
		Negate = CLIB.graphene_vec4_negate,
		GetY = CLIB.graphene_vec4_get_y,
		InitFromVec2 = CLIB.graphene_vec4_init_from_vec2,
		Normalize = CLIB.graphene_vec4_normalize,
	}
	META.__index = META
	ffi.metatype("struct _graphene_vec4_t", META)
end
do
	local META = {
		InitFromPoint = CLIB.graphene_plane_init_from_point,
		GetConstant = CLIB.graphene_plane_get_constant,
		__unm = CLIB.graphene_plane_negate,
		InitFromPoints = CLIB.graphene_plane_init_from_points,
		__gc = CLIB.graphene_plane_free,
		InitFromPlane = CLIB.graphene_plane_init_from_plane,
		Equal = CLIB.graphene_plane_equal,
		GetNormal = CLIB.graphene_plane_get_normal,
		Init = CLIB.graphene_plane_init,
		Free = CLIB.graphene_plane_free,
		Negate = CLIB.graphene_plane_negate,
		Distance = CLIB.graphene_plane_distance,
		InitFromVec4 = CLIB.graphene_plane_init_from_vec4,
		Normalize = CLIB.graphene_plane_normalize,
	}
	META.__index = META
	ffi.metatype("struct _graphene_plane_t", META)
end
do
	local META = {
		Translate = CLIB.graphene_matrix_translate,
		ProjectPoint = CLIB.graphene_matrix_project_point,
		ProjectRectBounds = CLIB.graphene_matrix_project_rect_bounds,
		SkewXy = CLIB.graphene_matrix_skew_xy,
		RotateQuaternion = CLIB.graphene_matrix_rotate_quaternion,
		Perspective = CLIB.graphene_matrix_perspective,
		Print = CLIB.graphene_matrix_print,
		TransformRect = CLIB.graphene_matrix_transform_rect,
		UnprojectPoint3d = CLIB.graphene_matrix_unproject_point3d,
		TransformPoint = CLIB.graphene_matrix_transform_point,
		InitFrom_2d = CLIB.graphene_matrix_init_from_2d,
		__mul = CLIB.graphene_matrix_multiply,
		Transpose = CLIB.graphene_matrix_transpose,
		InitFrustum = CLIB.graphene_matrix_init_frustum,
		RotateEuler = CLIB.graphene_matrix_rotate_euler,
		Is_2d = CLIB.graphene_matrix_is_2d,
		IsIdentity = CLIB.graphene_matrix_is_identity,
		InitFromMatrix = CLIB.graphene_matrix_init_from_matrix,
		InitFromFloat = CLIB.graphene_matrix_init_from_float,
		InitRotate = CLIB.graphene_matrix_init_rotate,
		RotateY = CLIB.graphene_matrix_rotate_y,
		IsBackfaceVisible = CLIB.graphene_matrix_is_backface_visible,
		InitPerspective = CLIB.graphene_matrix_init_perspective,
		InitSkew = CLIB.graphene_matrix_init_skew,
		SkewYz = CLIB.graphene_matrix_skew_yz,
		GetXScale = CLIB.graphene_matrix_get_x_scale,
		Rotate = CLIB.graphene_matrix_rotate,
		IsSingular = CLIB.graphene_matrix_is_singular,
		InitFromVec4 = CLIB.graphene_matrix_init_from_vec4,
		TransformVec3 = CLIB.graphene_matrix_transform_vec3,
		__gc = CLIB.graphene_matrix_free,
		TransformRay = CLIB.graphene_matrix_transform_ray,
		InitTranslate = CLIB.graphene_matrix_init_translate,
		InitScale = CLIB.graphene_matrix_init_scale,
		To_2d = CLIB.graphene_matrix_to_2d,
		TransformBounds = CLIB.graphene_matrix_transform_bounds,
		UntransformPoint = CLIB.graphene_matrix_untransform_point,
		GetYScale = CLIB.graphene_matrix_get_y_scale,
		ProjectRect = CLIB.graphene_matrix_project_rect,
		TransformBox = CLIB.graphene_matrix_transform_box,
		Interpolate = CLIB.graphene_matrix_interpolate,
		UntransformBounds = CLIB.graphene_matrix_untransform_bounds,
		GetZScale = CLIB.graphene_matrix_get_z_scale,
		InitLookAt = CLIB.graphene_matrix_init_look_at,
		GetRow = CLIB.graphene_matrix_get_row,
		Multiply = CLIB.graphene_matrix_multiply,
		ToFloat = CLIB.graphene_matrix_to_float,
		InitIdentity = CLIB.graphene_matrix_init_identity,
		InitOrtho = CLIB.graphene_matrix_init_ortho,
		TransformVec4 = CLIB.graphene_matrix_transform_vec4,
		Determinant = CLIB.graphene_matrix_determinant,
		TransformPoint3d = CLIB.graphene_matrix_transform_point3d,
		Scale = CLIB.graphene_matrix_scale,
		Inverse = CLIB.graphene_matrix_inverse,
		RotateX = CLIB.graphene_matrix_rotate_x,
		SkewXz = CLIB.graphene_matrix_skew_xz,
		TransformSphere = CLIB.graphene_matrix_transform_sphere,
		GetValue = CLIB.graphene_matrix_get_value,
		Free = CLIB.graphene_matrix_free,
		RotateZ = CLIB.graphene_matrix_rotate_z,
		Normalize = CLIB.graphene_matrix_normalize,
	}
	META.__index = META
	ffi.metatype("struct _graphene_matrix_t", META)
end
do
	local META = {
		__gc = CLIB.graphene_euler_free,
		InitFromVec3 = CLIB.graphene_euler_init_from_vec3,
		GetX = CLIB.graphene_euler_get_x,
		GetY = CLIB.graphene_euler_get_y,
		Reorder = CLIB.graphene_euler_reorder,
		GetOrder = CLIB.graphene_euler_get_order,
		ToVec3 = CLIB.graphene_euler_to_vec3,
		Free = CLIB.graphene_euler_free,
		Init = CLIB.graphene_euler_init,
		InitFromMatrix = CLIB.graphene_euler_init_from_matrix,
		InitFromQuaternion = CLIB.graphene_euler_init_from_quaternion,
		GetZ = CLIB.graphene_euler_get_z,
		InitFromEuler = CLIB.graphene_euler_init_from_euler,
		Equal = CLIB.graphene_euler_equal,
		InitWithOrder = CLIB.graphene_euler_init_with_order,
		ToMatrix = CLIB.graphene_euler_to_matrix,
	}
	META.__index = META
	ffi.metatype("struct _graphene_euler_t", META)
end
do
	local META = {
		__gc = CLIB.graphene_vec2_free,
		GetX = CLIB.graphene_vec2_get_x,
		Length = CLIB.graphene_vec2_length,
		__mul = CLIB.graphene_vec2_multiply,
		Equal = CLIB.graphene_vec2_equal,
		InitFromVec2 = CLIB.graphene_vec2_init_from_vec2,
		ToFloat = CLIB.graphene_vec2_to_float,
		Near = CLIB.graphene_vec2_near,
		Add = CLIB.graphene_vec2_add,
		Divide = CLIB.graphene_vec2_divide,
		__len = CLIB.graphene_vec2_length,
		Max = CLIB.graphene_vec2_max,
		__sub = CLIB.graphene_vec2_subtract,
		__unm = CLIB.graphene_vec2_negate,
		Dot = CLIB.graphene_vec2_dot,
		InitFromFloat = CLIB.graphene_vec2_init_from_float,
		__div = CLIB.graphene_vec2_divide,
		Min = CLIB.graphene_vec2_min,
		Scale = CLIB.graphene_vec2_scale,
		GetY = CLIB.graphene_vec2_get_y,
		Multiply = CLIB.graphene_vec2_multiply,
		__add = CLIB.graphene_vec2_add,
		Free = CLIB.graphene_vec2_free,
		Negate = CLIB.graphene_vec2_negate,
		Init = CLIB.graphene_vec2_init,
		Subtract = CLIB.graphene_vec2_subtract,
		Normalize = CLIB.graphene_vec2_normalize,
	}
	META.__index = META
	ffi.metatype("struct _graphene_vec2_t", META)
end
do
	local META = {
		Init = CLIB.graphene_ray_init,
		InitFromRay = CLIB.graphene_ray_init_from_ray,
		InitFromVec3 = CLIB.graphene_ray_init_from_vec3,
		GetClosestPointToPoint = CLIB.graphene_ray_get_closest_point_to_point,
		GetDistanceToPoint = CLIB.graphene_ray_get_distance_to_point,
		Equal = CLIB.graphene_ray_equal,
		__gc = CLIB.graphene_ray_free,
		GetDirection = CLIB.graphene_ray_get_direction,
		GetOrigin = CLIB.graphene_ray_get_origin,
		Free = CLIB.graphene_ray_free,
		GetDistanceToPlane = CLIB.graphene_ray_get_distance_to_plane,
		GetPositionAt = CLIB.graphene_ray_get_position_at,
	}
	META.__index = META
	ffi.metatype("struct _graphene_ray_t", META)
end
do
	local META = {
		GetCenter = CLIB.graphene_rect_get_center,
		Expand = CLIB.graphene_rect_expand,
		InitFromRect = CLIB.graphene_rect_init_from_rect,
		GetX = CLIB.graphene_rect_get_x,
		GetY = CLIB.graphene_rect_get_y,
		Union = CLIB.graphene_rect_union,
		ContainsRect = CLIB.graphene_rect_contains_rect,
		Equal = CLIB.graphene_rect_equal,
		Offset = CLIB.graphene_rect_offset,
		Interpolate = CLIB.graphene_rect_interpolate,
		Free = CLIB.graphene_rect_free,
		InsetR = CLIB.graphene_rect_inset_r,
		Inset = CLIB.graphene_rect_inset,
		Normalize = CLIB.graphene_rect_normalize,
		Init = CLIB.graphene_rect_init,
		GetBottomRight = CLIB.graphene_rect_get_bottom_right,
		NormalizeR = CLIB.graphene_rect_normalize_r,
		Round = CLIB.graphene_rect_round,
		GetWidth = CLIB.graphene_rect_get_width,
		RoundToPixel = CLIB.graphene_rect_round_to_pixel,
		GetTopRight = CLIB.graphene_rect_get_top_right,
		OffsetR = CLIB.graphene_rect_offset_r,
		Intersection = CLIB.graphene_rect_intersection,
		ContainsPoint = CLIB.graphene_rect_contains_point,
		GetTopLeft = CLIB.graphene_rect_get_top_left,
		__gc = CLIB.graphene_rect_free,
		GetBottomLeft = CLIB.graphene_rect_get_bottom_left,
		GetVertices = CLIB.graphene_rect_get_vertices,
		GetHeight = CLIB.graphene_rect_get_height,
	}
	META.__index = META
	ffi.metatype("struct _graphene_rect_t", META)
end
do
	local META = {
		Init = CLIB.graphene_size_init,
		Interpolate = CLIB.graphene_size_interpolate,
		Free = CLIB.graphene_size_free,
		InitFromSize = CLIB.graphene_size_init_from_size,
		__gc = CLIB.graphene_size_free,
		Equal = CLIB.graphene_size_equal,
		Scale = CLIB.graphene_size_scale,
	}
	META.__index = META
	ffi.metatype("struct _graphene_size_t", META)
end
do
	local META = {
		GetCenter = CLIB.graphene_box_get_center,
		Expand = CLIB.graphene_box_expand,
		InitFromVec3 = CLIB.graphene_box_init_from_vec3,
		Union = CLIB.graphene_box_union,
		Equal = CLIB.graphene_box_equal,
		GetSize = CLIB.graphene_box_get_size,
		Intersection = CLIB.graphene_box_intersection,
		Free = CLIB.graphene_box_free,
		ExpandVec3 = CLIB.graphene_box_expand_vec3,
		GetMin = CLIB.graphene_box_get_min,
		Init = CLIB.graphene_box_init,
		__gc = CLIB.graphene_box_free,
		GetVertices = CLIB.graphene_box_get_vertices,
		InitFromVectors = CLIB.graphene_box_init_from_vectors,
		InitFromBox = CLIB.graphene_box_init_from_box,
		ExpandScalar = CLIB.graphene_box_expand_scalar,
		ContainsPoint = CLIB.graphene_box_contains_point,
		InitFromPoints = CLIB.graphene_box_init_from_points,
		GetBoundingSphere = CLIB.graphene_box_get_bounding_sphere,
		GetMax = CLIB.graphene_box_get_max,
		GetDepth = CLIB.graphene_box_get_depth,
		GetWidth = CLIB.graphene_box_get_width,
		ContainsBox = CLIB.graphene_box_contains_box,
		GetHeight = CLIB.graphene_box_get_height,
	}
	META.__index = META
	ffi.metatype("struct _graphene_box_t", META)
end
do
	local META = {
		InitFromVec3 = CLIB.graphene_vec3_init_from_vec3,
		__mul = CLIB.graphene_vec3_multiply,
		Equal = CLIB.graphene_vec3_equal,
		GetXyzw = CLIB.graphene_vec3_get_xyzw,
		ToFloat = CLIB.graphene_vec3_to_float,
		__sub = CLIB.graphene_vec3_subtract,
		Add = CLIB.graphene_vec3_add,
		Init = CLIB.graphene_vec3_init,
		__unm = CLIB.graphene_vec3_negate,
		GetXyz0 = CLIB.graphene_vec3_get_xyz0,
		GetZ = CLIB.graphene_vec3_get_z,
		__gc = CLIB.graphene_vec3_free,
		GetX = CLIB.graphene_vec3_get_x,
		Length = CLIB.graphene_vec3_length,
		Cross = CLIB.graphene_vec3_cross,
		__div = CLIB.graphene_vec3_divide,
		Divide = CLIB.graphene_vec3_divide,
		__len = CLIB.graphene_vec3_length,
		GetXy0 = CLIB.graphene_vec3_get_xy0,
		InitFromFloat = CLIB.graphene_vec3_init_from_float,
		Free = CLIB.graphene_vec3_free,
		Dot = CLIB.graphene_vec3_dot,
		Near = CLIB.graphene_vec3_near,
		Multiply = CLIB.graphene_vec3_multiply,
		Min = CLIB.graphene_vec3_min,
		Scale = CLIB.graphene_vec3_scale,
		GetXyz1 = CLIB.graphene_vec3_get_xyz1,
		Max = CLIB.graphene_vec3_max,
		__add = CLIB.graphene_vec3_add,
		Subtract = CLIB.graphene_vec3_subtract,
		Negate = CLIB.graphene_vec3_negate,
		GetY = CLIB.graphene_vec3_get_y,
		GetXy = CLIB.graphene_vec3_get_xy,
		Normalize = CLIB.graphene_vec3_normalize,
	}
	META.__index = META
	ffi.metatype("struct _graphene_vec3_t", META)
end
do
	local META = {
		GetPoint = CLIB.graphene_quad_get_point,
		InitFromRect = CLIB.graphene_quad_init_from_rect,
		InitFromPoints = CLIB.graphene_quad_init_from_points,
		Bounds = CLIB.graphene_quad_bounds,
		__gc = CLIB.graphene_quad_free,
		Init = CLIB.graphene_quad_init,
		Free = CLIB.graphene_quad_free,
		Contains = CLIB.graphene_quad_contains,
	}
	META.__index = META
	ffi.metatype("struct _graphene_quad_t", META)
end
do
	local META = {
		Init = CLIB.graphene_frustum_init,
		InitFromMatrix = CLIB.graphene_frustum_init_from_matrix,
		InitFromFrustum = CLIB.graphene_frustum_init_from_frustum,
		IntersectsBox = CLIB.graphene_frustum_intersects_box,
		__gc = CLIB.graphene_frustum_free,
		ContainsPoint = CLIB.graphene_frustum_contains_point,
		IntersectsSphere = CLIB.graphene_frustum_intersects_sphere,
		Free = CLIB.graphene_frustum_free,
		GetPlanes = CLIB.graphene_frustum_get_planes,
	}
	META.__index = META
	ffi.metatype("struct _graphene_frustum_t", META)
end
do
	local META = {
		GetMidpoint = CLIB.graphene_triangle_get_midpoint,
		GetNormal = CLIB.graphene_triangle_get_normal,
		GetPlane = CLIB.graphene_triangle_get_plane,
		InitFromVec3 = CLIB.graphene_triangle_init_from_vec3,
		__gc = CLIB.graphene_triangle_free,
		GetArea = CLIB.graphene_triangle_get_area,
		Equal = CLIB.graphene_triangle_equal,
		ContainsPoint = CLIB.graphene_triangle_contains_point,
		InitFromPoint3d = CLIB.graphene_triangle_init_from_point3d,
		GetPoints = CLIB.graphene_triangle_get_points,
		Free = CLIB.graphene_triangle_free,
		GetBarycoords = CLIB.graphene_triangle_get_barycoords,
		GetBoundingBox = CLIB.graphene_triangle_get_bounding_box,
		GetVertices = CLIB.graphene_triangle_get_vertices,
	}
	META.__index = META
	ffi.metatype("struct _graphene_triangle_t", META)
end
do
	local META = {
		__gc = CLIB.graphene_quaternion_free,
		ToAngles = CLIB.graphene_quaternion_to_angles,
		InitFromEuler = CLIB.graphene_quaternion_init_from_euler,
		InitFromAngles = CLIB.graphene_quaternion_init_from_angles,
		InitIdentity = CLIB.graphene_quaternion_init_identity,
		Free = CLIB.graphene_quaternion_free,
		ToVec4 = CLIB.graphene_quaternion_to_vec4,
		ToAngleVec3 = CLIB.graphene_quaternion_to_angle_vec3,
		InitFromMatrix = CLIB.graphene_quaternion_init_from_matrix,
		Dot = CLIB.graphene_quaternion_dot,
		Slerp = CLIB.graphene_quaternion_slerp,
		Init = CLIB.graphene_quaternion_init,
		InitFromRadians = CLIB.graphene_quaternion_init_from_radians,
		Invert = CLIB.graphene_quaternion_invert,
		InitFromQuaternion = CLIB.graphene_quaternion_init_from_quaternion,
		ToMatrix = CLIB.graphene_quaternion_to_matrix,
		ToRadians = CLIB.graphene_quaternion_to_radians,
		InitFromAngleVec3 = CLIB.graphene_quaternion_init_from_angle_vec3,
		Equal = CLIB.graphene_quaternion_equal,
		InitFromVec4 = CLIB.graphene_quaternion_init_from_vec4,
		Normalize = CLIB.graphene_quaternion_normalize,
	}
	META.__index = META
	ffi.metatype("struct _graphene_quaternion_t", META)
end
do
	local META = {
		InitFromPoint = CLIB.graphene_point_init_from_point,
		Init = CLIB.graphene_point_init,
		__gc = CLIB.graphene_point_free,
		Equal = CLIB.graphene_point_equal,
		ToVec2 = CLIB.graphene_point_to_vec2,
		Interpolate = CLIB.graphene_point_interpolate,
		Free = CLIB.graphene_point_free,
		Near = CLIB.graphene_point_near,
		InitFromVec2 = CLIB.graphene_point_init_from_vec2,
		Distance = CLIB.graphene_point_distance,
	}
	META.__index = META
	ffi.metatype("struct _graphene_point_t", META)
end
do
	local META = {
		__gc = CLIB.graphene_point3d_free,
		InitFromVec3 = CLIB.graphene_point3d_init_from_vec3,
		Length = CLIB.graphene_point3d_length,
		Equal = CLIB.graphene_point3d_equal,
		Interpolate = CLIB.graphene_point3d_interpolate,
		Free = CLIB.graphene_point3d_free,
		Near = CLIB.graphene_point3d_near,
		Distance = CLIB.graphene_point3d_distance,
		__len = CLIB.graphene_point3d_length,
		InitFromPoint = CLIB.graphene_point3d_init_from_point,
		NormalizeViewport = CLIB.graphene_point3d_normalize_viewport,
		Dot = CLIB.graphene_point3d_dot,
		Scale = CLIB.graphene_point3d_scale,
		Init = CLIB.graphene_point3d_init,
		Cross = CLIB.graphene_point3d_cross,
		ToVec3 = CLIB.graphene_point3d_to_vec3,
		Normalize = CLIB.graphene_point3d_normalize,
	}
	META.__index = META
	ffi.metatype("struct _graphene_point3d_t", META)
end
do
	local META = {
		Init = CLIB.graphene_sphere_init,
		__gc = CLIB.graphene_sphere_free,
		IsEmpty = CLIB.graphene_sphere_is_empty,
		Translate = CLIB.graphene_sphere_translate,
		InitFromPoints = CLIB.graphene_sphere_init_from_points,
		GetRadius = CLIB.graphene_sphere_get_radius,
		GetBoundingBox = CLIB.graphene_sphere_get_bounding_box,
		ContainsPoint = CLIB.graphene_sphere_contains_point,
		GetCenter = CLIB.graphene_sphere_get_center,
		InitFromVectors = CLIB.graphene_sphere_init_from_vectors,
		Free = CLIB.graphene_sphere_free,
		Distance = CLIB.graphene_sphere_distance,
		Equal = CLIB.graphene_sphere_equal,
	}
	META.__index = META
	ffi.metatype("struct _graphene_sphere_t", META)
end
library.clib = CLIB
return library
