#ifndef ATLANTRONIC_TOOLS_H
#define ATLANTRONIC_TOOLS_H

#define EPSILON       0.00001f
#define STATIC_OBJ_MAX    100

struct atlantronic_vect2
{
	float x;
	float y;
};

struct atlantronic_vect3
{
	float x;           //!< position du robot (axe x, mm)
	float y;           //!< position du robot (axe y, mm)
	float theta;       //!< angle du robot (rd)
};

struct atlantronic_polyline
{
	int size;
	struct atlantronic_vect2* pt;
};

#define OBJECT_MOBILE               1         // objet mobile qu'on peut pousser
#define OBJECT_SEEN_BY_HOKUYO       2         // objet vu par le laser
#define OBJECT_SEEN_BY_OMRON        4         // objet vu par les omron

struct atlantronic_object
{
	int flags;
	struct atlantronic_polyline polyline;
};

float sat(float x, float min, float max);

void atlantronic_vect2_loc_to_abs(const struct atlantronic_vect3 *origin, const struct atlantronic_vect2 *pos_in, struct atlantronic_vect2 *pos_out);

void atlantronic_vect3_loc_to_abs(const struct atlantronic_vect3 *origin, const struct atlantronic_vect3 *pos_in, struct atlantronic_vect3 *pos_out);

struct atlantronic_vect3 atlantronic_vect3_loc_to_abs_speed(const double theta, const struct atlantronic_vect3* speed);

int atlantronic_segment_intersection(const struct atlantronic_vect2 a, const struct atlantronic_vect2 b, const struct atlantronic_vect2 c, const struct atlantronic_vect2 d, struct atlantronic_vect2* h);

float atlantronic_segment_distance(const struct atlantronic_vect2 a, const struct atlantronic_vect2 b, const struct atlantronic_vect2 c, const struct atlantronic_vect2 d);

void atlantronic_add_object(int flags, int size, struct atlantronic_vect2* pt);

void atlantronic_move_object(int id, struct atlantronic_vect2 origin, struct atlantronic_vect3 delta);

extern struct atlantronic_object atlantronic_static_obj[STATIC_OBJ_MAX];
extern int atlantronic_static_obj_count;

#endif
