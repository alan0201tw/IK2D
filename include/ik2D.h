#pragma once

#include "bone2D.h"
#include "target2D.h"

typedef enum ik_status_code
{
    IK_COMPLETE = 0,
    IK_ONGOING  = 1,
    IK_ERROR    = 2
} ik_status_code;

ik_status_code ik_inverse_jacobian(bone2D* b2d, target2D t2d);
ik_status_code ik_inverse_jacobian_single_iteration(bone2D* b2d, target2D t2d);

ik_status_code ik_ccd(bone2D* b2d, target2D t2d);
ik_status_code ik_ccd_single_iteration(bone2D* b2d, target2D t2d);