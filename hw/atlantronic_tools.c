#include "atlantronic_tools.h"
#include "kernel/robot_parameters.h"
#include "qemu-common.h"

// TODO voir pourquoi ca bug si on demarre avec atlantronic_static_obj_count = 0
// on met un objet loin pour contourner le probleme ...
struct atlantronic_vect2 bug[2] =
{
	{ -8000, -8000},
	{  8000, -8000}
};

struct atlantronic_polyline atlantronic_static_obj[STATIC_OBJ_MAX] =
{
	{ 2, bug },
};

int atlantronic_static_obj_count = 1;

const struct atlantronic_vect2 corner_loc[CORNER_NUM] =
{
	{ PARAM_RIGHT_CORNER_X / 65536.0f, PARAM_RIGHT_CORNER_Y / 65536.0f},
	{ PARAM_LEFT_CORNER_X / 65536.0f,  PARAM_LEFT_CORNER_Y / 65536.0f},
	{ PARAM_NP_X / 65536.0f,  PARAM_LEFT_CORNER_Y / 65536.0f},
	{ PARAM_NP_X / 65536.0f,  PARAM_RIGHT_CORNER_Y / 65536.0f},
};

void atlantronic_add_object(int size, struct atlantronic_vect2* pt)
{
	if( atlantronic_static_obj_count < STATIC_OBJ_MAX)
	{
		atlantronic_static_obj[atlantronic_static_obj_count].pt = g_malloc(sizeof(struct atlantronic_vect2) * size);
		memcpy(atlantronic_static_obj[atlantronic_static_obj_count].pt, pt, size * sizeof(struct atlantronic_vect2));
		atlantronic_static_obj[atlantronic_static_obj_count].size = size;
	}

	atlantronic_static_obj_count++;
}

void atlantronic_move_object(int id, struct atlantronic_vect2 origin, struct atlantronic_vect3 delta)
{
	int i;
	if( id < atlantronic_static_obj_count )
	{
		struct atlantronic_vect2* pt = atlantronic_static_obj[id].pt;
		delta.ca = cos(delta.alpha);
		delta.sa = sin(delta.alpha);

		for( i = 0; i < atlantronic_static_obj[id].size; i++)
		{
			float dx = pt[i].x - origin.x;
			float dy = pt[i].y - origin.y;
			pt[i].x = origin.x + dx * delta.ca - dy * delta.sa + delta.x;
			pt[i].y = origin.y + dx * delta.sa + dy * delta.ca + delta.y;
		}
	}
}

float sat(float x, float min, float max)
{
	if(x > max)
	{
		x = max;
	}
	else if(x < min)
	{
		x = min;
	}

	return x;
}

//! changement de repere du repère local au repere absolu
//! origin : origine du repère local dans le repère absolu
void atlantronic_vect2_loc_to_abs(const struct atlantronic_vect3 *origin, const struct atlantronic_vect2 *pos_in, struct atlantronic_vect2 *pos_out)
{
	pos_out->x = origin->x + origin->ca * pos_in->x - origin->sa * pos_in->y;
	pos_out->y = origin->y + origin->sa * pos_in->x + origin->ca * pos_in->y;
}

//! changement de repere du repère local au repere absolu
//! origin : origine du repère local dans le repère absolu
void atlantronic_vect3_loc_to_abs(const struct atlantronic_vect3 *origin, const struct atlantronic_vect3 *pos_in, struct atlantronic_vect3 *pos_out)
{
	pos_out->x = origin->x + origin->ca * pos_in->x - origin->sa * pos_in->y;
	pos_out->y = origin->y + origin->sa * pos_in->x + origin->ca * pos_in->y;
	pos_out->alpha = origin->alpha + pos_in->alpha;
	pos_out->ca = cos(pos_out->alpha);
	pos_out->sa = sin(pos_out->alpha);
}

//! calcule l'intersection h entre deux segments [a b] et [c d]
//! @return 0 si h est trouvé, < 0 sinon
int atlantronic_segment_intersection(const struct atlantronic_vect2 a, const struct atlantronic_vect2 b, const struct atlantronic_vect2 c, const struct atlantronic_vect2 d, struct atlantronic_vect2* h)
{
	double den = (b.y - a.y) * (d.x - c.x) - (d.y - c.y) * (b.x - a.x);
	double num = (d.x - c.x) * (c.y - a.y) - (d.y - c.y) * (c.x - a.x);

	if(den == 0)
	{
		// droites (a b) et (c d) parallèles
		return -1;
	}

	double alpha = num / den;

	if( fabs(alpha) < EPSILON )
	{
		alpha = 0;
	}

	if( fabs(alpha - 1) < EPSILON )
	{
		alpha = 1;
	}

	// on n'est pas dans [a b]
	if(alpha < 0 || alpha > 1)
	{
		return -1;
	}

	num = (b.y - a.y) * (a.x - c.x) - (b.x - a.x) * (a.y - c.y);

	double beta = num / den;

	if( fabs(beta) < EPSILON )
	{
		beta = 0;
	}

	if( fabs(beta - 1) < EPSILON )
	{
		beta = 1;
	}

	// on n'est pas dans [c d]
	if(beta < 0 || beta > 1)
	{
		return -2;
	}

	h->x = a.x + alpha * (b.x - a.x);
	h->y = a.y + alpha * (b.y - a.y);

	return 0;
}
