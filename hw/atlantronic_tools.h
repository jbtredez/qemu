#ifndef ATLANTRONIC_TOOLS_H
#define ATLANTRONIC_TOOLS_H

#define EPSILON      0.00001f
#define CORNER_NUM         4
#define STATIC_OBJ_NUM     9

struct atlantronic_vect2
{
	float x;
	float y;
};

struct atlantronic_vect3
{
	float x;           //!< position du robot (axe x, mm)
	float y;           //!< position du robot (axe y, mm)
	float alpha;       //!< angle du robot (rd)
	float ca;          //!< cos(alpha)
	float sa;          //!< sin(alpha)
};

struct atlantronic_polyline
{
	struct atlantronic_vect2* pt;
	int size;
};

float sat(float x, float min, float max);

void atlantronic_vect2_loc_to_abs(const struct atlantronic_vect3 *origin, const struct atlantronic_vect2 *pos_in, struct atlantronic_vect2 *pos_out);

void atlantronic_vect3_loc_to_abs(const struct atlantronic_vect3 *origin, const struct atlantronic_vect3 *pos_in, struct atlantronic_vect3 *pos_out);

int atlantronic_segment_intersection(const struct atlantronic_vect2 a, const struct atlantronic_vect2 b, const struct atlantronic_vect2 c, const struct atlantronic_vect2 d, struct atlantronic_vect2* h);

extern const struct atlantronic_vect2 corner_loc[CORNER_NUM];
extern const struct atlantronic_polyline atlantronic_static_obj[STATIC_OBJ_NUM];

#endif
