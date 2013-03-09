#include "atlantronic_tools.h"
#include "kernel/robot_parameters.h"

//!< contour de la table
const struct atlantronic_vect2 table_contour[5] =
{
	{ -1500, -1000}, { -1500, 1000 }, { 1500, 1000 }, { 1500, -1000 }, { -1500, -1000 }
};

//!< bordure diagonale zone marron coté bleu
const struct atlantronic_vect2 table_diag_border_blue[2] =
{
	{ -1175, -1000 }, { -1138, -251 }
};

//!< bordure case depart zone marron coté bleu
const struct atlantronic_vect2 table_start_border_blue[4] =
{
	{ -1500, 500 }, { -1000, 500 }, { -1000, 550 }, { -1500, 550 }
};

//!< bordure diagonale zone marron coté rouge
const struct atlantronic_vect2 table_diag_border_red[2] =
{
	{  1175, -1000 }, {  1138, -251 }
};

//!< bordure case depart zone marron coté rouge
const struct atlantronic_vect2 table_start_border_red[4] =
{
	{ 1500, 500 }, { 1000, 500 }, { 1000, 550 }, { 1500, 550 }
};

//!< totem coté bleu
const struct atlantronic_vect2 table_totem_blue[5] =
{
	{ -525, -125 }, { -275, -125 }, { -275, 125 }, { -525, 125 }, { -525, -125 }
};

//!< totem coté rouge
const struct atlantronic_vect2 table_totem_red[5] =
{
	{ 525, -125 }, { 275, -125 }, { 275, 125 }, { 525, 125 }, { 525, -125 }
};

//!< palmier (carre englobant)
const struct atlantronic_vect2 table_palm[5] =
{
	{ -75, -75 }, { 75, -75 }, { 75, 75 }, { -75, 75 }, { -75, -75 }
};

//!< totem coté rouge
const struct atlantronic_vect2 table_map[2] =
{
	{ -300, 990 }, { 300, 990 }
};

const struct atlantronic_polyline atlantronic_static_obj[STATIC_OBJ_NUM] =
{
	{ (struct atlantronic_vect2*) table_contour, 5 },
	{ (struct atlantronic_vect2*) table_diag_border_blue, 2 },
	{ (struct atlantronic_vect2*) table_start_border_blue, 4 },
	{ (struct atlantronic_vect2*) table_totem_blue, 5 },
	{ (struct atlantronic_vect2*) table_diag_border_red, 2 },
	{ (struct atlantronic_vect2*) table_start_border_red, 4 },
	{ (struct atlantronic_vect2*) table_totem_red, 5 },
	{ (struct atlantronic_vect2*) table_palm, 5},
	{ (struct atlantronic_vect2*) table_map, 2}
};

const struct atlantronic_vect2 corner_loc[CORNER_NUM] =
{
	{ PARAM_RIGHT_CORNER_X / 65536.0f, PARAM_RIGHT_CORNER_Y / 65536.0f},
	{ PARAM_LEFT_CORNER_X / 65536.0f,  PARAM_LEFT_CORNER_Y / 65536.0f},
	{ PARAM_NP_X / 65536.0f,  PARAM_RIGHT_CORNER_Y / 65536.0f},
	{ PARAM_NP_X / 65536.0f,  PARAM_LEFT_CORNER_Y / 65536.0f}
};

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
