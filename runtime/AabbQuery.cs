/*---------------------------------------------------------------------------------------------
 *  Copyright (c) Peter Bjorklund. All rights reserved.
 *  Licensed under the MIT License. See LICENSE in the project root for license information.
 *--------------------------------------------------------------------------------------------*/

using System.Runtime.CompilerServices;
using Unity.Mathematics;
using UnityEngine;

namespace Piot.Impact
{
    public struct Aabb
    {
        public float2 center;
        public float2 halfExtents;
    }

    public struct CollideResult
    {
        public float2 position;
        public float2 delta;
        public float2 normal;
        public float time;
    }

    public struct SweptResult
    {
        public CollideResult collideResult;
        public float2 position;
        public float time;
        public bool wasHit;
    }

    public static class AabbQuery
    {
        public static float SeparationCircle(Aabb box, float2 circleCenter, float circleRadius)
        {
            var centerDiff = circleCenter - box.center;
            var edgeDiff = math.abs(centerDiff) - box.halfExtents;

            return math.min(math.max(edgeDiff.x, edgeDiff.y), 0)
                   + math.length(math.max(edgeDiff, float2.zero))
                   - circleRadius;
        }

        public static bool OverlapCircle(Aabb box, float2 circleCenter, float circleRadius)
        {
            return SeparationCircle(box, circleCenter, circleRadius) <= 0f;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IntersectPoint(Aabb box, float2 point, out CollideResult collideResult)
        {
            var deltaX = point.x - box.center.x;
            var penetrationX = box.halfExtents.x - math.abs(deltaX);
            if (penetrationX < 0f)
            {
                collideResult = default;
                return false;
            }

            var deltaY = point.y - box.center.y;
            var penetrationY = box.halfExtents.y - math.abs(deltaY);
            if (penetrationY < 0f)
            {
                collideResult = default;
                return false;
            }

            collideResult = new CollideResult();
            if (penetrationX < penetrationY)
            {
                var signDeltaX = math.sign(deltaX);
                collideResult.delta.x = penetrationX * signDeltaX;
                collideResult.normal.x = signDeltaX;
                collideResult.position.x = box.center.x + box.halfExtents.x * signDeltaX;
                collideResult.position.y = point.y;
            }
            else
            {
                var signDeltaY = math.sign(deltaY);
                collideResult.delta.y = penetrationY * signDeltaY;
                collideResult.normal.y = signDeltaY;
                collideResult.position.x = point.x;
                collideResult.position.y = box.center.y + box.halfExtents.y * signDeltaY;
            }

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IntersectLineSegment(
            Aabb box,
            float2 position,
            float2 delta,
            float2 boxBorder,
            out CollideResult collideResult
        )
        {
            var scaleX = 1 / delta.x;
            var scaleY = 1 / delta.y;
            var signX = math.sign(scaleX);
            var signY = math.sign(scaleY);
            var nearTimeX =
                (box.center.x - signX * (box.halfExtents.x + boxBorder.x) - position.x) * scaleX;
            var nearTimeY =
                (box.center.y - signY * (box.halfExtents.y + boxBorder.y) - position.y) * scaleY;
            var farTimeX =
                (box.center.x + signX * (box.halfExtents.x + boxBorder.x) - position.x) * scaleX;
            var farTimeY =
                (box.center.y + signY * (box.halfExtents.y + boxBorder.y) - position.y) * scaleY;

            if (nearTimeX > farTimeY || nearTimeY > farTimeX)
            {
                collideResult = default;
                return false;
            }

            var nearTime = nearTimeX > nearTimeY ? nearTimeX : nearTimeY;
            var farTime = farTimeX < farTimeY ? farTimeX : farTimeY;
            if (nearTime > 1f || farTime < 0f)
            {
                collideResult = default;
                return false;
            }

            collideResult = new CollideResult
            {
                time = math.clamp(nearTime, 0, 1)
            };

            if (nearTimeX > nearTimeY)
            {
                collideResult.normal.x = -signX;
                collideResult.normal.y = 0f;
            }
            else
            {
                collideResult.normal.x = 0f;
                collideResult.normal.y = -signY;
            }

            collideResult.delta = (1.0f - collideResult.time) * -delta;
            collideResult.position = position + delta * collideResult.time;

            return true;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IntersectAabb(in Aabb boxA, in Aabb boxB, out CollideResult collideResult)
        {
            var deltaX = boxB.center.x - boxA.center.x;
            var penetrationX = boxB.halfExtents.x + boxA.halfExtents.x - math.abs(deltaX);
            if (penetrationX < 0f)
            {
                collideResult = default;
                return false;
            }

            var deltaY = boxB.center.y - boxA.center.y;
            var penetrationY = boxB.halfExtents.y + boxA.halfExtents.y - math.abs(deltaY);
            if (penetrationY < 0f)
            {
                collideResult = default;
                return false;
            }

            collideResult = new CollideResult();
            if (penetrationX < penetrationY)
            {
                var signDeltaX = math.sign(deltaX);
                collideResult.delta.x = penetrationX * signDeltaX;
                collideResult.normal.x = signDeltaX;
                collideResult.position.x = boxA.center.x + boxA.halfExtents.x * signDeltaX;
                collideResult.position.y = boxB.center.y;
            }
            else
            {
                var signDeltaY = math.sign(deltaY);
                collideResult.delta.y = penetrationY * signDeltaY;
                collideResult.normal.y = signDeltaY;
                collideResult.position.x = boxB.center.x;
                collideResult.position.y = boxA.center.y + boxA.halfExtents.y * signDeltaY;
            }

            return true;
        }

        const float TimeFactor = 0.99f;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static SweptResult SweptAabb(Aabb boxToSweep, Aabb targetBox, float2 delta)
        {
            var sweptResult = new SweptResult();


            // If the movement is extremely small, just make an overlap test instead
            // math.normalize will fail otherwise
            if (math.length(delta) <= math.EPSILON)
            {
                sweptResult.position = boxToSweep.center;
                var wasOverlap = IntersectAabb(targetBox, boxToSweep, out sweptResult.collideResult);
                sweptResult.time = wasOverlap ? 0f : 1f;
                sweptResult.wasHit = wasOverlap;
                return sweptResult;
            }

            var wasHit = IntersectLineSegment(targetBox, boxToSweep.center, delta, boxToSweep.halfExtents,
                out sweptResult.collideResult);
            if (wasHit)
            {
                sweptResult.time = math.clamp(sweptResult.collideResult.time, 0f, 1f);
                sweptResult.position = boxToSweep.center + (delta * sweptResult.time * TimeFactor);
                var direction = math.normalizesafe(delta);
                sweptResult.collideResult.position.x = math.clamp(
                    sweptResult.collideResult.position.x + direction.x * boxToSweep.halfExtents.x,
                    targetBox.center.x - targetBox.halfExtents.x,
                    targetBox.center.x + targetBox.halfExtents.x
                );
                sweptResult.collideResult.position.y = math.clamp(
                    sweptResult.collideResult.position.y + direction.y * boxToSweep.halfExtents.y,
                    targetBox.center.y - targetBox.halfExtents.y,
                    targetBox.center.y + targetBox.halfExtents.y
                );
            }
            else
            {
                sweptResult.time = TimeFactor;
                sweptResult.position = boxToSweep.center + delta * sweptResult.time;
            }

            sweptResult.wasHit = wasHit;

            return sweptResult;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static SweptResult ClosestSweptAabb(Aabb boxToMove, Aabb[] targetBoxes, float2 delta)
        {
            var closest = new SweptResult();

            // Assume that we could move the whole way
            closest.time = TimeFactor;
            closest.position = boxToMove.center + delta;

            foreach (var targetBox in targetBoxes)
            {
                var iterationSweep = SweptAabb(boxToMove, targetBox, delta);
                if (iterationSweep.time < closest.time)
                {
                    closest = iterationSweep;
                }
            }

            return closest;
        }
    }
}