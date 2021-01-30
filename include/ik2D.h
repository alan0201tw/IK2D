#pragma once

typedef enum ik_status_code
{
    IK_COMPLETE = 0,
    IK_ONGOING  = 1,
    IK_ERROR    = 2
} ik_status_code;

ik_status_code ik_inverse_jacobian();
ik_status_code ik_inverse_jacobian_single_iteration();

ik_status_code ik_ccd();
ik_status_code ik_ccd_single_iteration();