#include "ik2D.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "linmath.h"

#include "bone2D.h"
#include "target2D.h"

// FOR DEBUGGING
#include "renderer2D.h"

typedef struct __internal_world_space_position_and_rotation
{
    union
    {
        vec2 position;
        struct
        {
            float x, y;
        };
    };
    float angle, cos_value, sin_value;
} __internal_world_space_position_and_rotation;

ik_status_code ik_inverse_jacobian(bone2D* b2d, target2D t2d)
{
    ik_status_code code = IK_ONGOING;

    while (code == IK_ONGOING)
    {
        code = ik_inverse_jacobian_single_iteration(b2d, t2d);
    }

    return code;
}

ik_status_code ik_inverse_jacobian_single_iteration(bone2D* b2d, target2D t2d)
{
    const float epsilon = 0.001f;
    const float trivialArcLength = 0.00001f;

    bone2D* b2d_list[128] = { 0 };
    int bone_count = 0;
    while (b2d != NULL)
    {
        b2d_list[bone_count++] = b2d;
        b2d = b2d->parent;
    }

    // reverse the array
    for (int c = 0; c < bone_count / 2; c++)
    {
        bone2D* t = b2d_list[c];                  // Swapping
        b2d_list[c] = b2d_list[bone_count - c - 1];
        b2d_list[bone_count - c - 1] = t;
    }

    // index = 0 is the leaf bone, the last index is the root
    __internal_world_space_position_and_rotation world_bone_data[128];
    bone2D* root_bone = b2d_list[0];

    if (root_bone == NULL)
    {
        fprintf(stderr, "ERROR : Root_bone is null in ik_ccd_single_iteration!\n");
        return IK_ERROR;
    }

    world_bone_data[0].x = 0.0f;
    world_bone_data[0].y = 0.0f;
    world_bone_data[0].angle = root_bone->angle;
    world_bone_data[0].cos_value = cosf(root_bone->angle);
    world_bone_data[0].sin_value = sinf(root_bone->angle);

    for (int boneIdx = 1; boneIdx < bone_count; ++boneIdx)
    {
        __internal_world_space_position_and_rotation previous_world_bone
            = world_bone_data[boneIdx - 1];
        bone2D* current_bone = b2d_list[boneIdx];

        world_bone_data[boneIdx].angle = previous_world_bone.angle + current_bone->angle;
        world_bone_data[boneIdx].cos_value = cosf(world_bone_data[boneIdx].angle);
        world_bone_data[boneIdx].sin_value = sinf(world_bone_data[boneIdx].angle);
        world_bone_data[boneIdx].x =
            previous_world_bone.x + world_bone_data[boneIdx].cos_value * current_bone->length;
        world_bone_data[boneIdx].y =
            previous_world_bone.y + world_bone_data[boneIdx].sin_value * current_bone->length;

        render2D_render_point(world_bone_data[boneIdx].x, world_bone_data[boneIdx].y);
    }

    // Starting position of b2d_list[boneIdx] is world_bone_data[boneIdx - 1];
    // Ending position is world_bone_data[boneIdx - 1];
    // So to control the ending position of world_bone_data[boneIdx], we need to tune b2d_list[boneIdx + 1];
    // b2d_list[0] is root, with starting position = ending position = vec2(0.0f);

    vec2 end_effector_position;
    vec2_dup(end_effector_position, world_bone_data[bone_count - 1].position);

    // The size of jacobian matrix should be [dim, total_dof], 
    // which in this case should be [2, 1 * bone_count].
    vec2 jacobian_matrix[128]; // dim = [2, bone_count]

    // For pseudo-inverse we can just use transpose
    vec2 V;
    vec2_sub(V, t2d.position, end_effector_position);

    if (vec2_len(V) <= epsilon)
    {
        return IK_COMPLETE;
    }

    for (int boneIdx = 1; boneIdx < bone_count; ++boneIdx)
    {
        vec2 starting_position;
        vec2_dup(starting_position, world_bone_data[boneIdx - 1].position);
        vec2 arm_vector;
        vec2_sub(arm_vector, end_effector_position, starting_position);

        // extended_arm_vector
        vec3 extended_arm_vector = { arm_vector[0], arm_vector[1], 0.0f };
        vec3 rotation_axis = { 0.0f, 0.0f, 1.0f };
        vec3 jacobian_entry;
        vec3_mul_cross(jacobian_entry, rotation_axis, extended_arm_vector);

        jacobian_matrix[boneIdx - 1][0] = jacobian_entry[0];
        jacobian_matrix[boneIdx - 1][1] = jacobian_entry[1];
        // ignore the z-value of jacobian_entry
    }

    float delta_theta[128]; // dim = [bone_count, 1]
    for (int boneIdx = 1; boneIdx < bone_count; ++boneIdx)
    {
        //delta_theta[boneIdx - 1] = vec2_mul_inner(V, jacobian_matrix[boneIdx - 1]);
        delta_theta[boneIdx - 1] = vec2_mul_inner(V, jacobian_matrix[boneIdx - 1]);
        b2d_list[boneIdx]->angle += delta_theta[boneIdx - 1];
    }

    return IK_ONGOING;
}

ik_status_code ik_ccd(bone2D* b2d, target2D t2d)
{
    ik_status_code code = IK_ONGOING;

    while (code == IK_ONGOING)
    {
        code = ik_ccd_single_iteration(b2d, t2d);
    }

    return code;
}

ik_status_code ik_ccd_single_iteration(bone2D* b2d, target2D t2d)
{
    // Ref : https://www.ryanjuckett.com/cyclic-coordinate-descent-in-2d/
    const float epsilon = 0.001f;
    const float trivialArcLength = 0.00001f;

    bone2D* b2d_list[128] = { 0 };
    int bone_count = 0;
    while(b2d != NULL)
    {
        b2d_list[bone_count++] = b2d;
        b2d = b2d->parent;
    }

    // reverse the array
    for (int c = 0; c < bone_count/2; c++)
    {
        bone2D* t = b2d_list[c];                  // Swapping
        b2d_list[c] = b2d_list[bone_count-c-1];
        b2d_list[bone_count-c-1] = t;
    }

    // index = 0 is the leaf bone, the last index is the root
    __internal_world_space_position_and_rotation world_bone_data[128];
    bone2D* root_bone = b2d_list[0];

    if (root_bone == NULL)
    {
        fprintf(stderr, "ERROR : Root_bone is null in ik_ccd_single_iteration!\n");
        return IK_ERROR;
    }

    world_bone_data[0].x = 0.0f;
    world_bone_data[0].y = 0.0f;
    world_bone_data[0].angle = root_bone->angle;
    world_bone_data[0].cos_value = cosf(root_bone->angle);
    world_bone_data[0].sin_value = sinf(root_bone->angle);

    render2D_render_point(world_bone_data[0].x, world_bone_data[0].y);

    for(int boneIdx = 1; boneIdx < bone_count; ++boneIdx)
    {
        __internal_world_space_position_and_rotation previous_world_bone 
            = world_bone_data[boneIdx - 1];
        bone2D* current_bone = b2d_list[boneIdx];
 
        world_bone_data[boneIdx].angle = previous_world_bone.angle + current_bone->angle;
        world_bone_data[boneIdx].cos_value = cosf( world_bone_data[boneIdx].angle );
        world_bone_data[boneIdx].sin_value = sinf( world_bone_data[boneIdx].angle );
        world_bone_data[boneIdx].x = 
            previous_world_bone.x + world_bone_data[boneIdx].cos_value * current_bone->length;
        world_bone_data[boneIdx].y = 
            previous_world_bone.y + world_bone_data[boneIdx].sin_value * current_bone->length;

        render2D_render_point(world_bone_data[boneIdx].x, world_bone_data[boneIdx].y);
    }

    // Starting position of b2d_list[boneIdx] is world_bone_data[boneIdx - 1];
    // Ending position is world_bone_data[boneIdx - 1];
    // So to control the ending position of world_bone_data[boneIdx], we need to tune b2d_list[boneIdx + 1];
    // b2d_list[0] is root, with starting position = ending position = vec2(0.0f);

    vec2 end_effector_position;
    end_effector_position[0] = world_bone_data[bone_count-1].x;
    end_effector_position[1] = world_bone_data[bone_count-1].y;

    bool modifiedBones = false;
    for (int boneIdx = bone_count - 2; boneIdx >= 0; --boneIdx)
    {
        // Get the vector from the current bone to the end effector position.
        vec2 curToEnd;
        vec2_sub(curToEnd, end_effector_position, world_bone_data[boneIdx].position);
        float curToEndMag = vec2_len(curToEnd);

        // Get the vector from the current bone to the target position.
        vec2 curToTarget;
        vec2_sub(curToTarget, t2d.position, world_bone_data[boneIdx].position);
        float curToTargetMag = vec2_len(curToTarget);

        // Get rotation to place the end effector on the line from the current
        // joint position to the target postion.
        float cosRotAng;
        float sinRotAng;
        float endTargetMag = (curToEndMag * curToTargetMag);
        if (endTargetMag <= epsilon)
        {
            cosRotAng = 1.0f;
            sinRotAng = 0.0f;
        }
        else
        {
            cosRotAng = (curToEnd[0] * curToTarget[0] + curToEnd[1] * curToTarget[1]) / endTargetMag;
            sinRotAng = (curToEnd[0] * curToTarget[1] - curToEnd[1] * curToTarget[0]) / endTargetMag;
        }

        // Clamp the cosine into range when computing the angle (might be out of range
        // due to floating point error).
        float rotAng = acosf(fmaxf(-1.0f, fminf(1.0f, cosRotAng)));
        if (sinRotAng < 0.0f)
            rotAng = -rotAng;

        // Rotate the end effector position.
        end_effector_position[0] = world_bone_data[boneIdx].x + cosRotAng * curToEnd[0] - sinRotAng * curToEnd[1];
        end_effector_position[1] = world_bone_data[boneIdx].y + sinRotAng * curToEnd[0] + cosRotAng * curToEnd[1];

        // Rotate the current bone in local space (this value is output to the user)
        b2d_list[boneIdx + 1]->angle += rotAng;
        // fprintf(stderr, "idx = %d : add ang = %f \n", boneIdx, rotAng);

        // Check for termination
        float endToTargetX = (t2d.x - end_effector_position[0]);
        float endToTargetY = (t2d.y - end_effector_position[1]);
        if (endToTargetX * endToTargetX + endToTargetY * endToTargetY <= 0.000001f)
        {
            // We found a valid solution.
            return IK_COMPLETE;
        }

        // Track if the arc length that we moved the end effector was
        // a nontrivial distance.
        if (!modifiedBones && fabsf(rotAng) * curToEndMag > trivialArcLength)
        {
            modifiedBones = true;
        }
    }

    if( modifiedBones )
        return IK_ONGOING;
    else
        return IK_ERROR;
}