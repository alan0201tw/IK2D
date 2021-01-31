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

ik_status_code ik_inverse_jacobian(bone2D* b2d_list, target2D t2d)
{

}

ik_status_code ik_inverse_jacobian_single_iteration(bone2D* b2d_list, target2D t2d)
{

}

ik_status_code ik_ccd(bone2D* b2d, target2D t2d)
{
    ik_status_code code;

    do
    {
        ik_ccd_single_iteration(b2d, t2d);
    } while (code == IK_ONGOING);
    
    return code;
}

typedef struct __internal_world_space_position_and_rotation
{
    float x, y;
    float angle, cos_value, sin_value;
} __internal_world_space_position_and_rotation;

ik_status_code ik_ccd_single_iteration(bone2D* b2d, target2D t2d)
{
    // Ref : https://www.ryanjuckett.com/cyclic-coordinate-descent-in-2d/
    const float epsilon = 0.001f;
    const float trivialArcLength = 0.00001f;

    bone2D* b2d_list[128];
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

    vec2 end_effector_position;
    end_effector_position[0] = world_bone_data[bone_count-1].x;
    end_effector_position[1] = world_bone_data[bone_count-1].y;

    bool modifiedBones = false;
    for(int boneIdx = bone_count-2; boneIdx >= 0; --boneIdx)
    {
        // Get the vector from the current bone to the end effector position.
        float curToEndX = end_effector_position[0] - world_bone_data[boneIdx].x;
        float curToEndY = end_effector_position[1] - world_bone_data[boneIdx].y;
        float curToEndMag = sqrtf( curToEndX*curToEndX + curToEndY*curToEndY );

        // Get the vector from the current bone to the target position.
        float curToTargetX = t2d.x - world_bone_data[boneIdx].x;
        float curToTargetY = t2d.y - world_bone_data[boneIdx].y;
        float curToTargetMag = sqrtf(curToTargetX*curToTargetX + curToTargetY*curToTargetY);

        // Get rotation to place the end effector on the line from the current
        // joint position to the target postion.
        float cosRotAng;
        float sinRotAng;
        float endTargetMag = (curToEndMag*curToTargetMag);
        if( endTargetMag <= epsilon )
        {
            cosRotAng = 1.0f;
            sinRotAng = 0.0f;
        }
        else
        {
            cosRotAng = (curToEndX*curToTargetX + curToEndY*curToTargetY) / endTargetMag;
            sinRotAng = (curToEndX*curToTargetY - curToEndY*curToTargetX) / endTargetMag;
        }

        // Clamp the cosine into range when computing the angle (might be out of range
        // due to floating point error).
        float rotAng = acosf(fmaxf(-1.0f, fminf(1.0f, cosRotAng)));
        if( sinRotAng < 0.0f )
            rotAng = -rotAng;

        // Rotate the end effector position.
        end_effector_position[0] = world_bone_data[boneIdx].x + cosRotAng*curToEndX - sinRotAng*curToEndY;
        end_effector_position[1] = world_bone_data[boneIdx].y + sinRotAng*curToEndX + cosRotAng*curToEndY;

        // Rotate the current bone in local space (this value is output to the user)
        b2d_list[boneIdx+1]->angle += rotAng;
        // fprintf(stderr, "idx = %d : add ang = %f \n", boneIdx, rotAng);

        // Check for termination
        float endToTargetX = (t2d.x - end_effector_position[0]);
        float endToTargetY = (t2d.y - end_effector_position[1]);
        if( endToTargetX*endToTargetX + endToTargetY*endToTargetY <= 0.000001f )
        {
            // We found a valid solution.
            return IK_COMPLETE;
        }
    
        // Track if the arc length that we moved the end effector was
        // a nontrivial distance.
        if( !modifiedBones && fabsf(rotAng)*curToEndMag > trivialArcLength )
        {
            modifiedBones = true;
        }
    }

    if( modifiedBones )
        return IK_ONGOING;
    else
        return IK_ERROR;
}